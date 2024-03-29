/*
 * Copyright (C) 2021, HENSOLDT Cyber GmbH
 */
#include "OS_FileSystem.h"
#include "lib_debug/Debug.h"
#include "OS_Socket.h"
#include "interfaces/if_OS_Socket.h"
#include <string.h>
#include <camkes.h>

#include <unistd.h>
#include <math.h>

#include "OS_Error.h"
//#include "if_i2c.h"
#include "i2c_lib.h"
//#include "mpu6050.h"
#include <stdint.h>

// Assign the RPC endpoint based on the names used by this client
/**
 * @todo store the points in a dictionary ?
 * @todo maybe transfer the absolute position from the python interface to Rpi?
 * @todo make the code neat!!!
 */

/*

sudo sdk/scripts/open_trentos_build_env.sh sdk/build-system.sh sdk/demos/demo_tum_course rpi3 build-rpi3-Debug-demo_tum_course-task5 -DCMAKE_BUILD_TYPE=Debug

*/
#define POINT_CLOUD_ROW 500
#define POINT_CLOUD_CLN 3
#define POINT_CLOUD_NUM 3 * 500
#define POINT_CLOUD_SIZE sizeof(double) * 3 * 500
#define NOT_A_NUM (__builtin_nanf(""))
#define DISTANCE_THRESHOLD 33*33

static char fileData[4096];
static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);

/**
 * @brief an instance of the obstacle block
 * @param number: the number of points that belongs to this block set
 *
 */
struct block
{
    double x ;
    double y ;
    double z ;
    int number;
};

// by default 10 blocks
uint8_t block_number = 0;
struct block blocks[100];
double cur_pos[3] = {0,0,0};

double euclidean_distance(double x1, double y1, double x2, double y2)
{
    // Calculating distance
    // return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
    return (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
}


// ----------------------------- mpu6050-related events START------------------------------------
#define DEVICE (0x68 << 1) // MPU6050 device id

/**
 * Read the accel/gyro data, and create feedback to the Airsim.
 */
uint16_t accelX, accelY, accelZ;
uint16_t gyroX, gyroY, gyroZ;
char feedback = 'K'; // a "command" that can control the speed of the drone

char get_feedback_from_accel_gyro(uint16_t *accelX, uint16_t *accelY, uint16_t *accelZ, uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ)
{
    mpu6050_rpc_get_data(accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    /* printf("TestApp.c: accel_X=%d, accelY=%d, accelZ=%d, gyro_X=%d, gyroY=%d, gyroZ=%d\n", \
         accelX-32768, accelY-32768, accelZ-32768, gyroX-32768, gyroY-32768, gyroZ-32768); */
    if(*accelX > 12500 && *accelX < 18000) // make the drone slower
    {
        return 'S';
    }
    else if(*accelX > 47500 && *accelX < 53000) // make the drone faster
    {
        return 'F';
    }
    else // keep the speed unchanged
    {
        return 'K';
    }
}

/**
 * Initialize the mpu6050 sensor.
 */
OS_Error_t run_i2c(void)
{
    OS_Error_t err = OS_SUCCESS;
    I2C_Error_t i2c_err = I2C_SUCCESS;
    size_t tmp = 0;

    while(!mpu6050_rpc_sensor_ready())
    {
        seL4_Yield();
    }

    if_I2C_t bus = IF_I2C_ASSIGN(i2c_rpc, i2c_port, i2cBus_notify);    
    uint8_t buf[20];
    buf[0] = 0xd0;
    i2c_init_slave(&bus, DEVICE);

    i2c_err = i2c_write(&bus, DEVICE, 1, &tmp, buf);
    if(i2c_err != I2C_SUCCESS)
    {
        Debug_LOG_ERROR("i2c_write() returned errorcode %d", i2c_err);
        err = OS_ERROR_ABORTED;
        return err;
    }

    i2c_err = i2c_read(&bus, DEVICE, 1, &tmp, buf);
    if(i2c_err != I2C_SUCCESS)
    {
        Debug_LOG_ERROR("i2c_read() returned errorcode %d", i2c_err);
        err = OS_ERROR_ABORTED;
        return err;
    }   

    return err;
}
// ----------------------------- mpu6050-related events END ------------------------------------


void bubble_sort(struct block blocks[], int block_length)
{
    struct block temp;
    for (int i = 0; i < block_length - 1; ++i)
        for (int j = i + 1; j < block_length; ++j)
        {
            if (blocks[i].z <= blocks[j].z) 
            {
                temp = blocks[i];
                blocks[i] = blocks[j];
                blocks[j] = temp;
            }
        }
}


//------------------------------------------------------------------------------
static OS_FileSystem_Config_t spiffsCfg_1 =
    {
        .type = OS_FileSystem_Type_SPIFFS,
        .size = OS_FileSystem_USE_STORAGE_MAX,
        .storage = IF_OS_STORAGE_ASSIGN(
            storage_rpc_1,
            storage_dp_1),
};
static OS_FileSystem_Config_t spiffsCfg_2 =
    {
        .type = OS_FileSystem_Type_SPIFFS,
        .size = OS_FileSystem_USE_STORAGE_MAX,
        .storage = IF_OS_STORAGE_ASSIGN(
            storage_rpc_2,
            storage_dp_2),
};

//------------------------------------------------------------------------------
static void
test_OS_FileSystem(OS_FileSystem_Config_t *cfg)
{
    OS_Error_t ret;
    OS_FileSystem_Handle_t hFs;
    static const char *fileName = "testfile.txt";
    // static const uint8_t fileData[] =
    //     {
    //         0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    static const off_t fileSize = sizeof(fileData);
    static OS_FileSystemFile_Handle_t hFile;

    // Init file system
    if ((ret = OS_FileSystem_init(&hFs, cfg)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystem_init() failed, code %d", ret);
    }

    // Format file system
    if ((ret = OS_FileSystem_format(hFs)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystem_format() failed, code %d", ret);
    }
    // Mount file system
    if ((ret = OS_FileSystem_mount(hFs)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystem_mount() failed, code %d", ret);
    }

    // Open file
    if ((ret = OS_FileSystemFile_open(hFs, &hFile, fileName,
                                      OS_FileSystem_OpenMode_RDWR,
                                      OS_FileSystem_OpenFlags_CREATE)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystemFile_open() failed, code %d", ret);
    }

    // Write to the file
    off_t to_write, written;
    to_write = fileSize;
    written = 0;
    while (to_write > 0)
    {
        if ((ret = OS_FileSystemFile_write(hFs, hFile, written, sizeof(fileData), fileData)) != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_FileSystemFile_write() failed, code %d", ret);
        }
        written += sizeof(fileData);
        to_write -= sizeof(fileData);
    }

    // Read from the file
    uint8_t buf[sizeof(fileData)];
    off_t to_read, read;
    to_read = fileSize;
    read = 0;
    while (to_read > 0)
    {
        if ((ret = OS_FileSystemFile_read(hFs, hFile, read, sizeof(buf), buf)) != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_FileSystemFile_read() failed, code %d", ret);
        }
        if (memcmp(fileData, buf, sizeof(buf)))
            Debug_LOG_ERROR("File content read does not equal file content to be written.");
        read += sizeof(buf);
        to_read -= sizeof(buf);
    }

    // Close file
    if ((ret = OS_FileSystemFile_close(hFs, hFile)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystemFile_close() failed, code %d", ret);
    }
    // Clean up
    if ((ret = OS_FileSystem_unmount(hFs)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystem_unmount() failed, code %d", ret);
    }
    if ((ret = OS_FileSystem_free(hFs)) != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_FileSystem_free() failed, code %d", ret);
    }
}

static OS_Error_t
waitForNetworkStackInit(
    const if_OS_Socket_t *const ctx)
{
    OS_NetworkStack_State_t networkStackState;
    for (;;)
    {
        networkStackState = OS_Socket_getStatus(ctx);
        if (networkStackState == RUNNING)
        {
            // NetworkStack up and running.
            return OS_SUCCESS;
        }
        else if (networkStackState == FATAL_ERROR)
        {
            // NetworkStack will not come up.
            Debug_LOG_ERROR("A FATAL_ERROR occurred in the Network Stack component.");
            return OS_ERROR_ABORTED;
        }
        // Yield to wait until the stack is up and running.
        seL4_Yield();
    }
}

static OS_Error_t
waitForIncomingConnection(
    const int srvHandleId)
{
    OS_Error_t ret;

    // Wait for the event letting us know that the connection was successfully
    // established.
    // for (;;)
    // {
    ret = OS_Socket_wait(&networkStackCtx);
    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_wait() failed, code %d", ret);
        // break;
    }
    // char evtBuffer[128];
    // const size_t evtBufferSize = sizeof(evtBuffer);
    // int numberOfSocketsWithEvents;
    // ret = OS_Socket_getPendingEvents(
    //     &networkStackCtx,
    //     evtBuffer,
    //     evtBufferSize,
    //     &numberOfSocketsWithEvents);
    // if (ret != OS_SUCCESS)
    // {
    //     Debug_LOG_ERROR("OS_Socket_getPendingEvents() failed, code %d",
    //                     ret);
    //     break;
    // }
    // if (numberOfSocketsWithEvents == 0)
    // {
    //     Debug_LOG_TRACE("OS_Socket_getPendingEvents() returned "
    //                     "without any pending events");
    //     continue;
    // }
    // // We only opened one socket, so if we get more events, this is not ok.
    // if (numberOfSocketsWithEvents != 1)
    // {
    //     Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned with "
    //                     "unexpected #events: %d",
    //                     numberOfSocketsWithEvents);
    //     ret = OS_ERROR_INVALID_STATE;
    //     break;
    // }
    // OS_Socket_Evt_t event;
    // memcpy(&event, evtBuffer, sizeof(event));
    // if (event.socketHandle != srvHandleId)
    // {
    //     Debug_LOG_ERROR("Unexpected handle received: %d, expected: %d",
    //                     event.socketHandle, srvHandleId);
    //     ret = OS_ERROR_INVALID_HANDLE;
    //     break;
    // }
    // // Socket has been closed by NetworkStack component.
    // if (event.eventMask & OS_SOCK_EV_FIN)
    // {
    //     Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
    //                     "OS_SOCK_EV_FIN for handle: %d",
    //                     event.socketHandle);
    //     ret = OS_ERROR_NETWORK_CONN_REFUSED;
    //     break;
    // }
    // // Incoming connection received.
    // if (event.eventMask & OS_SOCK_EV_CONN_ACPT)
    // {
    //     Debug_LOG_DEBUG("OS_Socket_getPendingEvents() returned "
    //                     "connection established for handle: %d",
    //                     event.socketHandle);
    //     ret = OS_SUCCESS;

    //     break;
    // }
    // // Remote socket requested to be closed only valid for clients.
    // if (event.eventMask & OS_SOCK_EV_CLOSE)
    // {
    //     Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
    //                     "OS_SOCK_EV_CLOSE for handle: %d",
    //                     event.socketHandle);
    //     ret = OS_ERROR_CONNECTION_CLOSED;
    //     break;
    // }
    // // Error received - print error.
    // if (event.eventMask & OS_SOCK_EV_ERROR)
    // {
    //     Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
    //                     "OS_SOCK_EV_ERROR for handle: %d, code: %d",
    //                     event.socketHandle, event.currentError);
    //     ret = event.currentError;
    //     break;
    // }
    // }
    return ret;
}

/**
 * @brief 
 * 
 * @param str 
 * @param delimiters 
 * @param tokens 
 * @return the length of the point cloud 
 */

int tokenize_string(char str[], char delimiters[], double cloud_points[][3], double cur_pos[3])
{

    char *p = str;
    int i;
    for (i = 0; p /*if ptr not a null*/; i++)
    {

        p = strtok(i == 0 ? str : NULL, delimiters); // only the first call pass str
        cloud_points[i / POINT_CLOUD_CLN][i % POINT_CLOUD_CLN] = (p) ? atof(p) : NOT_A_NUM;
    }

    // pass the value tocurrent position 
    cur_pos[0] = cloud_points[i / POINT_CLOUD_CLN-1][0];
    cur_pos[1] = cloud_points[i / POINT_CLOUD_CLN-1][1];
    cur_pos[2] = cloud_points[i / POINT_CLOUD_CLN-1][2];
    
    // clear the last row of tokens
    cloud_points[i / POINT_CLOUD_CLN-1][0] = NOT_A_NUM;
    cloud_points[i / POINT_CLOUD_CLN-1][1] = NOT_A_NUM;
    cloud_points[i / POINT_CLOUD_CLN-1][2] = NOT_A_NUM;

    return i / POINT_CLOUD_CLN -1;
}

//------------------------------------------------------------------------------

int run()
{
    printf("Starting test_app_server...");
    // Check and wait until the NetworkStack component is up and running.
    OS_Error_t ret = waitForNetworkStackInit(&networkStackCtx);
    if (OS_SUCCESS != ret)
    {
        Debug_LOG_ERROR("waitForNetworkStackInit() failed with: %d", ret);
        return -1;
    }
    OS_Socket_Handle_t hServer;
    ret = OS_Socket_create(
        &networkStackCtx,
        &hServer,
        OS_AF_INET,
        OS_SOCK_STREAM);

    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_create() failed, code %d", ret);
        return -1;
    }
    const OS_Socket_Addr_t dstAddr =
        {
            .addr = CFG_TEST_HTTP_SERVER,
            .port = EXERCISE_SERVER_PORT};

    //------------------------------Connect establishing STARTS ----------------------
    ret = OS_Socket_connect(hServer, &dstAddr);
    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_connect() failed, code %d", ret);
        OS_Socket_close(hServer);
        return -1;
    }

    static uint8_t receivedData[OS_DATAPORT_DEFAULT_SIZE];
    printf("OS_DATAPORT_DEFAULT_SIZE %lu", OS_DATAPORT_DEFAULT_SIZE);

    do
    {
        ret = waitForIncomingConnection(hServer.handleID);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("waitForIncomingConnection() failed, error %d", ret);
            OS_Socket_close(hServer);
            return -1;
        }

    } while (ret == OS_ERROR_TRY_AGAIN);

    //------------------------------Connect establishing ENDS ----------------------

    //------------------------------SEND REQUEST TO HOST STARTS ----------------------
    printf("Send request to host...");
    const char request[] =
        "GET / HTTP/1.0\r\nHost:10.0.0.1\r\nConnection: close\r\n";
    size_t actualLen = sizeof(request);
    size_t to_write = strlen(request);

    char request_land[256] = "LAND\r\n";

    do
    {
        seL4_Yield();
        ret = OS_Socket_write(hServer, request, to_write, &actualLen);
    } while (ret == OS_ERROR_WOULD_BLOCK);

    if (OS_SUCCESS != ret)
    {
        Debug_LOG_ERROR("OS_Socket_write() failed with error code %d", ret);
        OS_Socket_close(hServer);
        return -1;
    }
    // printf("HTTP request successfully sent");

    /**
     * @todo put these variables somewhere else
     *
     */
    size_t actualLenRecv = 0;
    char very_long_tmp[0xffff];
    int string_good = 1;
    int counter = 0;
    double lidar_points[POINT_CLOUD_ROW][POINT_CLOUD_CLN];
    double pre_lidar_points[POINT_CLOUD_ROW][POINT_CLOUD_CLN];
    int lidar_points_len, pre_lidar_points_len;
    char str_tem[32];

    run_i2c(); 

    while (1)
    {

        while (string_good)
        {
            ret = OS_Socket_read(
                hServer,
                receivedData,
                sizeof(receivedData),
                &actualLenRecv);

            /**
             * @todo uncomment these if we want to use Debug_LOG_INFO
             *
             */
            // Debug_LOG_INFO("string_good--------------------------------ret:%d", ret);
            // Debug_LOG_INFO("fileData[actualLenRecv - 1]:%d", fileData[actualLenRecv - 1]);
            // Debug_LOG_INFO("actualLenRecv:%d", actualLenRecv);

            memcpy(fileData, receivedData, sizeof(fileData));
            strcat(very_long_tmp, fileData);
            if (fileData[actualLenRecv - 1] == '\0' && actualLenRecv)
            {
                string_good = 0;
            }
            memset(receivedData, 0, sizeof(receivedData));
        }

        lidar_points_len = tokenize_string(very_long_tmp, " ,[]\n", lidar_points,cur_pos);
        printf("The number of detected points is: %d\r\n", lidar_points_len);
        // printf("Got HTTP Page:\n%s\r\n", very_long_tmp);

        if (lidar_points_len == 0)
        {
            counter++;
        }

        /*If we know there are three times that the lidar points are zero,
          we confirm that there is no obstacles in the near
        */

        if (counter > 3)
        {
            printf("Fly to Destination and landing\r\n");

            /**
             * @todo temporarily print the stored data points
             *
             */

            // calculate the mean of the relative x and y value
            double cur_x = 0;
            double cur_y = 0;
            for (int i = 0; i < pre_lidar_points_len; i++)
            {
                cur_x += pre_lidar_points[i][0];
                cur_y += pre_lidar_points[i][1];
            }
            cur_x /= pre_lidar_points_len;
            cur_y /= pre_lidar_points_len;

            
            /* slightly modify the destination to avoid landing deviation */
            sprintf(str_tem, "%f %f %f\r\n", (cur_x*1.02)+cur_pos[0], (cur_y*1.02)+cur_pos[1],cur_pos[2]-1);
            strcat(request_land, str_tem);
            size_t actualLen_land = sizeof(request_land);
            size_t to_write_land = strlen(request_land);

            /* send the landing infomation to the drone */
            do
            {
                seL4_Yield();
                ret = OS_Socket_write(hServer, request_land, to_write_land, &actualLen_land);
            } while (ret == OS_ERROR_WOULD_BLOCK);

            /* wait for the "FINISH" message from the server */
            do
            {
                ret = OS_Socket_read(
                hServer,
                receivedData,
                sizeof(receivedData),
                &actualLenRecv);
            } while (actualLenRecv <= 4);

            /* sort the block heights */
            bubble_sort(blocks, block_number);

            /* print the sorted block heights */
            for (int i = 0; i < block_number; i++)
            {
                printf("Block %d  x:%f y:%f height:%f number: %d\n", (i + 1), blocks[i].x, blocks[i].y, blocks[i].z, blocks[i].number);
            }

            /**
             * @todo the break should be changed to something else later
             *
             */
            break;
        }
        else
        { /*else fly in vertical direction*/
            // printf("Fly in vertical direction\r\n");

            // if there is lidar points, record it for the landing destination
            if (lidar_points_len > 10)
            {
                // copy the current lidar points
                memcpy(pre_lidar_points, lidar_points, POINT_CLOUD_SIZE);
                pre_lidar_points_len = lidar_points_len;
            }

            /**
             * @brief record the blocks center points
             *
             */

            int cluster_found = 0;

            // loop the lidar pointss
            for (int j = 0; j < lidar_points_len; j++)
            {

                for (int i = 0; i < block_number; i++)
                {
                    // printf("euclidean_distance: %f\n",euclidean_distance(lidar_points[j][0], lidar_points[j][1], blocks[i].x, blocks[i].y));
                    if (euclidean_distance(lidar_points[j][0], lidar_points[j][1], blocks[i].x, blocks[i].y) < DISTANCE_THRESHOLD)
                    {
                        blocks[i].x = (lidar_points[j][0] + blocks[i].x * blocks[i].number) / (blocks[i].number + 1);
                        blocks[i].y = (lidar_points[j][1] + blocks[i].y * blocks[i].number) / (blocks[i].number + 1);
                        /**
                         * @todo z has to be changed later on
                         *
                         */
                        // printf("-----n if\n");
                        blocks[i].z = -cur_pos[2];;
                        blocks[i].number++;
                        cluster_found = 1;
                        break;
                    }

                    cluster_found = 0;
                }

                if (!cluster_found)
                {
                    blocks[block_number].x = lidar_points[j][0];
                    blocks[block_number].y = lidar_points[j][1];
                    blocks[block_number].z = -cur_pos[2];
                    blocks[block_number].number = 1;
                    block_number += 1;
                    // printf("number of blocks: %d\n", block_number);
                }
            }

            /* ----------------- get, handle the accel/gyro data START ------------------*/

            /* read the accel/gyro data, get the feedback */
            feedback = get_feedback_from_accel_gyro(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);

            /* send the feedback to the Python script */
            if(feedback == 'F')  // talk to the drone to move faster
            {
                const char request_fast[] = "FAST\r\n";
                size_t actualLen_fast = sizeof(request_fast);
                size_t to_write_fast = strlen(request_fast);
                do
                {
                    seL4_Yield();
                    ret = OS_Socket_write(hServer, request_fast, to_write_fast, &actualLen_fast);
                } while (ret == OS_ERROR_WOULD_BLOCK);
            }
            else if(feedback == 'S') // talk to the drone to move slower
            {
                const char request_slow[] = "SLOW\r\n";
                size_t actualLen_slow = sizeof(request_slow);
                size_t to_write_slow = strlen(request_slow);
                do
                {
                    seL4_Yield();
                    ret = OS_Socket_write(hServer, request_slow, to_write_slow, &actualLen_slow);
                } while (ret == OS_ERROR_WOULD_BLOCK);
            }
            else // talk to the drone to keep the speed unchanged
            {
                const char request_keep[] = "KEEP\r\n";
                size_t actualLen_keep = sizeof(request_keep);
                size_t to_write_keep = strlen(request_keep);
                do
                {
                    seL4_Yield();
                    ret = OS_Socket_write(hServer, request_keep, to_write_keep, &actualLen_keep);
                } while (ret == OS_ERROR_WOULD_BLOCK);
            }
        }

        /* ----------------- get, handle the accel/gyro data END ------------------*/

        strcpy(very_long_tmp, "");
        string_good = 1;
        actualLenRecv = 0;
        memset(receivedData, 0, sizeof(receivedData));
    }

    /* wait */
    for (int i = 0; i < 100000 * 2000; i++)
    {
        __asm__("nop");
    }


    OS_Socket_close(hServer);

    // ----------------------------------------------------------------------
    // Storage Test
    // ----------------------------------------------------------------------
    test_OS_FileSystem(&spiffsCfg_1);
    test_OS_FileSystem(&spiffsCfg_2);
    printf("Demo completed successfully.");
    return 0;
}
