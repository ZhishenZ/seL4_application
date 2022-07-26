#pragma once

#include "if_i2c.h"

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Called when post_init is execuited
 * 
 * @details Function to initialise underlying hardware components and software
 *          libraries. Is called once when the component initialises. Only
 *          after this function does the I2C interface send ready signal.
 * 
 * @return Return true if initialisation was successfull otherwise false.
 */
bool _i2c_init(void);

/**
 * @brief Initialise a new driver internal IC on the bus
 * 
 * @details Some Driver implementation have a contex object for each IC on the
 *          Bus. This function allows the wrapper to performe such an action.
 * 
 * @param dev address of the device on the I2C bus.
 * 
 * @return I2C_SUCCESS if registration was successfull, otherwise an error. If
 *         not implemented on the architecture return I2C_ERROR_NOT_IMPLEMENTED
 */
I2C_Error_t _i2c_init_slave(int dev);

/**
 * @brief Write buffer to I2C bus.
 * 
 * @details This function should write the buffer to the I2C peripheral or
 *          call a librarie to do so.
 * 
 * @param dev address of the device on the I2C bus.
 * 
 * @param buf Pointer to buffer containing the data to be written
 * 
 * @param len The number of bytes to be send on the bus. 
 * 
 * @param written Returns the actually written bytes on the bus.
 * 
 * @return Implementation specific.
 * 
 */
I2C_Error_t _i2c_write(int dev, const uint8_t* buf, size_t len, size_t* written);

/**
 * @brief Read bytes from the I2C peripheral to the buffer
 * 
 * @details This function should read the number of requested bytes from the
 *          I2C Bus to the buffer.
 * 
 * @param dev address of the device on the I2C bus.
 * 
 * @param buf Pointer to buffer were the data should be read to.
 * 
 * @param len The number of bytes to be read from the bus. 
 * 
 * @param written Returns the actually read bytes from the bus.
 * 
 * @return Implementation specific.
 */
I2C_Error_t _i2c_read(int dev, uint8_t *buf, size_t len, size_t *read);