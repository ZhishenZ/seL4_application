/*
* OS libraries configurations
*
* Copyright (C) 2021, HENSOLDT Cyber GmbH
*/
#pragma once
//-----------------------------------------------------------------------------
// Debug
//-----------------------------------------------------------------------------
#if !defined(NDEBUG)
#define Debug_Config_STANDARD_ASSERT
#define Debug_Config_ASSERT_SELF_PTR
#else
#define Debug_Config_DISABLE_ASSERT
#define Debug_Config_NO_ASSERT_SELF_PTR
#endif
#define Debug_Config_LOG_LEVEL Debug_LOG_LEVEL_INFO
#define Debug_Config_INCLUDE_LEVEL_IN_MSG
#define Debug_Config_LOG_WITH_FILE_LINE 
//-----------------------------------------------------------------------------
// Memory
//-----------------------------------------------------------------------------
#define Memory_Config_USE_STDLIB_ALLOC

//-----------------------------------------------------------------------------
// StorageServer
//-----------------------------------------------------------------------------
#define FILESYSTEM_1_STORAGE_OFFSET 0
#define FILESYSTEM_1_STORAGE_SIZE (32 * 1024) 
#define FILESYSTEM_2_STORAGE_OFFSET (32 * 1024)
#define FILESYSTEM_2_STORAGE_SIZE (32 * 1024)

//-----------------------------------------------------------------------------
// Network Stack
//-----------------------------------------------------------------------------
#define OS_NETWORK_MAXIMUM_SOCKET_NO 1
#define ETH_ADDR "10.0.0.11"
#define ETH_GATEWAY_ADDR "10.0.0.1"
#define ETH_SUBNET_MASK "255.255.255.0"
#define EXERCISE_SERVER_PORT 8888
#define CFG_TEST_HTTP_SERVER "10.0.0.10"

//-----------------------------------------------------------------------------
// NIC driver
//-----------------------------------------------------------------------------
#define NIC_DRIVER_RINGBUFFER_NUMBER_ELEMENTS 16
#define NIC_DRIVER_RINGBUFFER_SIZE \
    (NIC_DRIVER_RINGBUFFER_NUMBER_ELEMENTS * 4096)







