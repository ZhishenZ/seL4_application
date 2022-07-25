


#pragma once

#include <stdint.h>
#include <stddef.h>
#include "OS_Dataport.h"


/**
 * @brief Error codes used by this component
 * 
 */
typedef enum {
    I2C_ERROR_TRY_AGAIN = -9,
    I2C_ERROR_MUTEX_LOCKED,
    I2C_ERROR_NOT_IMPLEMENTED,
    I2C_ERROR_INVALID_PARAMETER,
    I2C_ERROR_NOT_INITIALISED,
    I2C_ERROR_INVALID_ADDRESS,
    I2C_ERROR_CLK_TIMEOUT,
    I2C_ERROR_NACK,
    I2C_ERROR_GENERIC,

    //---------------------------
    I2C_SUCCESS
}
I2C_Error_t;

/**
 * @brief Data structure used to manage the different potential I2C Interfaces
 *        in Use. Needed by the i2c_lib
 */
typedef struct
{
    OS_Dataport_t port_storage;
    I2C_Error_t (*write)(int dev, size_t len, size_t* write);
    I2C_Error_t (*read)(int dev, size_t len, size_t* read);
    I2C_Error_t (*mutex_try_lock)(void);
    I2C_Error_t (*mutex_unlock)(void);
    I2C_Error_t (*init_slave)(int dev);
    void (*notify_wait)(void);
} if_I2C_t;

/**
 * @brief Initialise the management struct of the i2c_lib with the correct
 *        function, port, and event pointer
 */
#define IF_I2C_ASSIGN(_rpc_, _port_, _evt_)    \
{                                              \
    .port_storage = OS_DATAPORT_ASSIGN(_port_), \
    .write = _rpc_ ## _write,                  \
    .read  = _rpc_ ##_read,                    \
    .mutex_try_lock = _rpc_ ## _mutex_try_lock,\
    .mutex_unlock = _rpc_ ## _mutex_unlock,    \
    .init_slave = _rpc_ ## _init_slave,        \
    .notify_wait = _evt_ ## _wait,             \
}

