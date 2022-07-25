/*
* Copyright (C) 2021, HENSOLDT Cyber GmbH
*/
#include "OS_FileSystem.h"
#include "OS_Socket.h"
#include "interfaces/if_OS_Socket.h"
#include "lib_debug/Debug.h"
#include <string.h>
#include <camkes.h>

#include "OS_Error.h"
//#include "if_i2c.h"
#include "i2c_lib.h"
//#include "mpu6050.h"
#include <stdint.h>

static char fileData[250];
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

// ---------mpu6050-------------
#define DEVICE (0x68 << 1)
void wait(void)
{
    for (int i = 0; i < 100000 * 400; i++)
    {
        __asm__("nop");
    }
}

void runDemo( void)
{
    uint16_t accelX, accelY, accelZ;
    uint16_t gyroX, gyroY, gyroZ;

    while(1)
    {
        mpu6050_rpc_get_data(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
        printf("TestApp.c: accel_X=%d, accelY=%d, accelZ=%d, gyro_X=%d, gyroY=%d, gyroZ=%d\n", \
            accelX-32768, accelY-32768, accelZ-32768, gyroX-32768, gyroY-32768, gyroZ-32768);
        wait();
    }
}

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

    // Debug_LOG_INFO("ID of bmp280 is 0x%x", buf[0]);
    // buf[0] = 0xf4;
    //  i2c_err = i2c_write(&bus, DEVICE, 1, &tmp, buf);
    // if(i2c_err != I2C_SUCCESS)
    // {
    //     Debug_LOG_ERROR("i2c_write() returned errorcode %d", i2c_err);
    //     err = OS_ERROR_ABORTED;
    //     return err;
    // }

    // i2c_err = i2c_read(&bus, DEVICE, 1, &tmp, buf);
    // if(i2c_err != I2C_SUCCESS)
    // {
    //     Debug_LOG_ERROR("i2c_read() returned errorcode %d", i2c_err);
    //     err = OS_ERROR_ABORTED;
    //     return err;
    // }  

    // Debug_LOG_INFO("ctrl_meas dump is 0x%x", buf[0]);

    Debug_LOG_INFO("Running Demo now");
    runDemo();
    
    Debug_LOG_INFO("Done");
    return err;
}


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

//----------------------------------------------------------------------
// Network
//----------------------------------------------------------------------
static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);
//------------------------------------------------------------------------------
static OS_Error_t waitForNetworkStackInit(
    const if_OS_Socket_t* const ctx)
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
    for (int i=0;i<1;i++)
    {
        ret = OS_Socket_wait(&networkStackCtx);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_Socket_wait() failed, code %d", ret);
            break;
        }

        char evtBuffer[128];
        const size_t evtBufferSize = sizeof(evtBuffer);
        int numberOfSocketsWithEvents;

        ret = OS_Socket_getPendingEvents(
            &networkStackCtx,
            evtBuffer,
            evtBufferSize,
            &numberOfSocketsWithEvents);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() failed, code %d", ret);
            break;
        }

        if (numberOfSocketsWithEvents == 0)
        {
            Debug_LOG_TRACE("OS_Socket_getPendingEvents() returned "
            "without any pending events");
            continue;
        }

        // We only opened one socket, so if we get more events, this is not ok.
        if (numberOfSocketsWithEvents != 1)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned with "
                            "unexpected #events: %d", numberOfSocketsWithEvents);
            ret = OS_ERROR_INVALID_STATE;
            break;
        }

        OS_Socket_Evt_t event;
        memcpy(&event, evtBuffer, sizeof(event));

        if (event.socketHandle != srvHandleId)
        {
            Debug_LOG_ERROR("Unexpected handle received: %d, expected: %d",
            event.socketHandle, srvHandleId);
            ret = OS_ERROR_INVALID_HANDLE;
            break;
        }

        // Socket has been closed by NetworkStack component.
        if (event.eventMask & OS_SOCK_EV_FIN)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
            "OS_SOCK_EV_FIN for handle: %d",
            event.socketHandle);
            ret = OS_ERROR_NETWORK_CONN_REFUSED;
            break;
        }

        // Incoming connection received.
        if (event.eventMask & OS_SOCK_EV_CONN_ACPT)
        {
            Debug_LOG_DEBUG("OS_Socket_getPendingEvents() returned "
            "connection established for handle: %d",
            event.socketHandle);
            ret = OS_SUCCESS;
            break;
        }

        // Remote socket requested to be closed only valid for clients.
        if (event.eventMask & OS_SOCK_EV_CLOSE)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
            "OS_SOCK_EV_CLOSE for handle: %d",
            event.socketHandle);
            ret = OS_ERROR_CONNECTION_CLOSED;
            break;
        }

        // Error received - print error.
        if (event.eventMask & OS_SOCK_EV_ERROR)
        {
            Debug_LOG_ERROR("OS_Socket_getPendingEvents() returned "
            "OS_SOCK_EV_ERROR for handle: %d, code: %d",
            event.socketHandle, event.currentError);
            ret = event.currentError;
            break;
        }
    }

    return ret;
}




//------------------------------------------------------------------------------
int run()
{
    Debug_LOG_INFO("Starting test_app_server...");
    // Check and wait until the NetworkStack component is up and running.
    OS_Error_t ret = waitForNetworkStackInit(&networkStackCtx);
    if (OS_SUCCESS != ret)
    {
        Debug_LOG_ERROR("waitForNetworkStackInit() failed with: %d", ret);
        return -1;
    }

    OS_Socket_Handle_t hServer;
    // ==================create starts===================
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
    // ==================create ends===================

    const OS_Socket_Addr_t dstAddr =
    {
        .addr = CFG_TEST_HTTP_SERVER,
        .port = EXERCISE_SERVER_PORT
    };

    // ==================connect starts===================
    ret = OS_Socket_connect(hServer, &dstAddr);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("OS_Socket_connect() failed, code %d", ret);
            OS_Socket_close(hServer);
            return ret;
        }
    // ==================connect ends===================

    // // =============================bind starts===================
    // ret = OS_Socket_bind(
    //     hServer,
    //     &dstAddr);
    // if (ret != OS_SUCCESS)
    // {
    //     Debug_LOG_ERROR("OS_Socket_bind() failed, code %d", ret);
    //     OS_Socket_close(hServer);
    //     return -1;
    // }
    // // =============================bind ends===================

    // // ==================listen starts===================
    // ret = OS_Socket_listen(
    // hServer,
    // 1);
    // if (ret != OS_SUCCESS)
    // {
    //     Debug_LOG_ERROR("OS_Socket_listen() failed, code %d", ret);
    //     OS_Socket_close(hServer);
    //     return -1;
    // }
    // // ==================listen ends===================

    static uint8_t receivedData[OS_DATAPORT_DEFAULT_SIZE];

    // Debug_LOG_INFO("Accepting new connection");
    // OS_Socket_Handle_t hSocket;
    // OS_Socket_Addr_t srcAddr = {0};

    do
    {
        ret = waitForIncomingConnection(hServer.handleID);
        if (ret != OS_SUCCESS)
        {
            Debug_LOG_ERROR("waitForIncomingConnection() failed, error %d", ret);
            OS_Socket_close(hServer);
            return -1;
        }
        // ret = OS_Socket_accept(
        // hServer,
        // &hSocket,
        // &srcAddr);
    }
    while (ret == OS_ERROR_TRY_AGAIN);
    // if (ret != OS_SUCCESS)
    // {
    //     Debug_LOG_ERROR("OS_Socket_accept() failed, error %d", ret);
    //     OS_Socket_close(hSocket);
    //     return -1;
    // }

    Debug_LOG_INFO("Send request to host...");

    const char request[] =
        "GET / HTTP/1.0\r\nHost: 10.0.0.1\r\n"
        "Connection: close\r\n\r\n";

    run_i2c();

    size_t lenWritten = sizeof(request);
    
    do
    {
        seL4_Yield();
        ret = OS_Socket_write(hServer, request, sizeof(request), &lenWritten);
    }
    while (ret == OS_ERROR_WOULD_BLOCK);

    if (ret != OS_SUCCESS)
    {
        Debug_LOG_ERROR("OS_Socket_write() failed with error code %d", ret);
        return -1;
    }
    Debug_LOG_INFO("HTTP request successfully sent");

    // Loop until an error occurs.
    do
    {
        size_t actualLenRecv = 0;
        ret = OS_Socket_read(
        hServer,
        receivedData,
        sizeof(receivedData),
        &actualLenRecv);
        switch (ret)
        {
        case OS_SUCCESS:
            Debug_LOG_INFO(
            "OS_Socket_read() - bytes read: %zu, err: %d",
            actualLenRecv, ret);
            memcpy(fileData, receivedData, sizeof(fileData));
            
            // print index.html
            Debug_LOG_INFO("Got HTTP Page:");
            printf("%s\r\n", fileData);
            
            // size_t lenWritten = 0;

            // ret = OS_Socket_write(
            // hSocket,
            // receivedData,
            // sizeof(fileData),
            // &lenWritten);
            // continue;

        case OS_ERROR_TRY_AGAIN:
            Debug_LOG_TRACE(
            "OS_Socket_read() reported try again");
            break;;
        
        case OS_ERROR_CONNECTION_CLOSED:
            Debug_LOG_INFO(
            "OS_Socket_read() reported connection closed");
            break;
        
        case OS_ERROR_NETWORK_CONN_SHUTDOWN:
            Debug_LOG_DEBUG(
            "OS_Socket_read() reported connection closed");
            break;
        
        default:
            Debug_LOG_ERROR(
            "OS_Socket_read() failed, error %d", ret);
            break;
        }
    }
    while (ret == OS_SUCCESS);
    OS_Socket_close(hServer);

    // ----------------------------------------------------------------------
    // Storage Test
    // ----------------------------------------------------------------------

    // Work on file system 1 (residing on partition 1)
    test_OS_FileSystem(&spiffsCfg_1);
    // Work on file system 2 (residing on partition 2)
    test_OS_FileSystem(&spiffsCfg_2);
    Debug_LOG_INFO("Demo completed successfully.");
    return 0;
}