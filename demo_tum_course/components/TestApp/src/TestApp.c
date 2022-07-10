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

// Assign the RPC endpoint based on the names used by this client

static char fileData[4096];
static const if_OS_Socket_t networkStackCtx = IF_OS_SOCKET_ASSIGN(networkStack);
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
    Debug_LOG_INFO("OS_DATAPORT_DEFAULT_SIZE %lu", OS_DATAPORT_DEFAULT_SIZE);

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
    Debug_LOG_INFO("Send request to host...");
    const char request[] =
        "GET / HTTP/1.0\r\nHost:10.0.0.1\r\nConnection: close\r\n";
    size_t actualLen = sizeof(request);
    size_t to_write = strlen(request);
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
    Debug_LOG_INFO("HTTP request successfully sent");

    //------------------------------SEND REQUEST TO HOST ENDS ----------------------

    // Loop until an error occurs.do
    // {

    //     size_t actualLenRecv = 0;
    //     ret = OS_Socket_read(
    //         hServer,
    //         receivedData,
    //         sizeof(receivedData),
    //         &actualLenRecv);
    //     switch (ret)
    //     {
    //     case OS_SUCCESS:
    //         Debug_LOG_INFO(
    //             "OS_Socket_read() received %zu bytes of data",
    //             actualLenRecv);
    //         memcpy(fileData, receivedData, sizeof(fileData));
    //         Debug_LOG_INFO("Got HTTP Page:\n%s\r\n", fileData);
    //         break;
    //         // size_t lenWritten = 0;
    //         // ret = OS_Socket_write(
    //         //     hServer,
    //         //     receivedData,
    //         //     sizeof(fileData),
    //         //     &lenWritten);
    //         // continue;
    //     case OS_ERROR_TRY_AGAIN:
    //         Debug_LOG_TRACE(
    //             "OS_Socket_read() reported try again");
    //         continue;
    //     case OS_ERROR_CONNECTION_CLOSED:
    //         Debug_LOG_INFO(
    //             "OS_Socket_read() reported connection closed");
    //         break;
    //     case OS_ERROR_NETWORK_CONN_SHUTDOWN:
    //         Debug_LOG_DEBUG(
    //             "OS_Socket_read() reported connection closed");
    //         break;
    //     default:
    //         Debug_LOG_ERROR(
    //             "OS_Socket_read() failed, error %d", ret);
    //         break;
    //     }
    // } while (ret == OS_ERROR_TRY_AGAIN);

    // Debug_LOG_INFO("STARTS wait for 1 second");

    // int a = 0;
    // // TimeServer_sleep(&timer, TimerServer_PRECISION_SEC,1);
    // for (int iii = 0; iii < 0xfffffff; iii++)
    // {
    //     a++;
    // }
    // Debug_LOG_INFO("ENDS wait for 1 second");

    size_t actualLenRecv = 0;
    char very_long_tmp[0xffff];
    int string_good = 1;

    // ret = OS_ERROR_TRY_AGAIN;

    // while ( ret == OS_ERROR_TRY_AGAIN)
    // {
    while (string_good)
    {
        ret = OS_Socket_read(
            hServer,
            receivedData,
            sizeof(receivedData),
            &actualLenRecv);

        Debug_LOG_INFO("string_good--------------------------------ret:%d", ret);
        Debug_LOG_INFO("fileData[actualLenRecv - 1]:%d", fileData[actualLenRecv - 1]);
        Debug_LOG_INFO("actualLenRecv:%d", actualLenRecv);
        memcpy(fileData, receivedData, sizeof(fileData));
        strcat(very_long_tmp, fileData);
        if (fileData[actualLenRecv - 1] == '\0' && actualLenRecv)
        {

            string_good = 0;
        }
    }
    // }

    Debug_LOG_INFO("Got HTTP Page:\n%s\r\n", very_long_tmp);

    // ret = OS_Socket_read(
    //     hServer,
    //     receivedData,
    //     sizeof(receivedData),
    //     &actualLenRecv);

    // Debug_LOG_INFO(
    //     "OS_Socket_read() received %zu bytes of data",
    //     actualLenRecv);

    // memcpy(fileData, receivedData, sizeof(fileData));
    // Debug_LOG_INFO("Got HTTP Page:\n%s\r\n", fileData);

    // printf("Last char of data :fileData[actualLenRecv-0] %c\n", fileData[actualLenRecv - 0]);
    // printf("Last char of data :fileData[actualLenRecv-1] %c\n", fileData[actualLenRecv - 1]);
    // printf("Last char of data :fileData[actualLenRecv-2] %c\n", fileData[actualLenRecv - 2]);
    // printf("Last char of data :fileData[actualLenRecv-3] %c\n", fileData[actualLenRecv - 3]);

    // ret = OS_Socket_read(
    //     hServer,
    //     receivedData,
    //     sizeof(receivedData),
    //     &actualLenRecv);

    // Debug_LOG_INFO(
    //     "OS_Socket_read() received %zu bytes of data",
    //     actualLenRecv);

    // memcpy(fileData, receivedData, sizeof(fileData));
    // Debug_LOG_INFO("Got HTTP Page:\n%s\r\n", fileData);

    // printf("Last char of data :fileData[actualLenRecv-0] %c\n", fileData[actualLenRecv - 0]);
    // printf("Last char of data :fileData[actualLenRecv-1] %c\n", fileData[actualLenRecv - 1]);
    // printf("Last char of data :fileData[actualLenRecv-2] %c\n", fileData[actualLenRecv - 2]);
    // printf("Last char of data :fileData[actualLenRecv-3] %c\n", fileData[actualLenRecv - 3]);

    OS_Socket_close(hServer);

    // ----------------------------------------------------------------------
    // Storage Test
    // ----------------------------------------------------------------------
    test_OS_FileSystem(&spiffsCfg_1);
    test_OS_FileSystem(&spiffsCfg_2);
    Debug_LOG_INFO("Demo completed successfully.");
    return 0;
}
