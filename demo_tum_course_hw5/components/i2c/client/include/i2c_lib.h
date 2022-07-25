

#include "if_i2c.h"

#include <stdint.h>


/**
 * @brief Helper function to wait for initialisation of I2C bus component
 * 
 * @details SeL4 does not guarantee that the I2C bus peripheral and driver
 *          already are correctly initialised. This function blocks until
 *          the driver issues an initialisation event.
 * 
 * @param bus Pointer to the if_i2c management struct, that allows multiple
 *            i2c components to be connected to one component.  
 * 
 * @return I2C_SUCCESS when I2C initialisation is finished, I2C_ERROR_GENERIC
 *         if the driver is not behaving correctly 
 */
I2C_Error_t i2c_wait_for_bus_initialisation(const if_I2C_t* bus);

/**
 * @brief Register slave on i2c Bus.
 * 
 * @details This function is needed on some platforms. It registers the 
 *          ic inside into an internal data structure. Other Architectures
 *          will implement a dummy, so it is save to be called.
 * 
 * @param bus Pointer to the if_i2c management struct, that allows multiple
 *            i2c components to be connected to one component.  
 *
 * @param dev I2C address of the desired IC. Needs to be even, as the lowest bit
 *            of the address is a read/write flag on the bus. If only 7 bits are
 *            provided by the datasheet, shift it by one position to the left.
 * 
 * @return Either I2C_SUCCESS if implemented by driver or I2C_ERROR_NOT_IMPLEMENTED.
 */
I2C_Error_t i2c_init_slave(const if_I2C_t* bus, int dev);


/**
 * @brief Write number of bytes to the I2C bus.
 *
 * @param bus Pointer to the if_i2c management struct, that allows multiple
 *            i2c components to be connected to one component.  
 *
 * @param dev I2C address of the desired IC. Needs to be even, as the lowest bit
 *            of the address is a read/write flag on the bus. If only 7 bits are
 *            provided by the datasheet, shift it by one position to the left.
 * 
 * @param len Number of bytes to be written from buf.
 * 
 * @param written pointer that returns how many bytes were actually written to
 *                the Bus
 * 
 * @param buf Pointer to buffer containing the bytes to be written.
 * 
 * @return Implementation sepcific.
 * 
 */
I2C_Error_t i2c_write(const if_I2C_t* bus,
                      int dev,
                      size_t len,
                      size_t *written,
                      const uint8_t* buf);

/**
 * @brief Write number of bytes to the IC at an IC internal register.
 * 
 * @details The reg argument is written before the bytes from buf on the Bus.
 *          Required for some ICs
 
 * @param bus Pointer to the if_i2c management struct, that allows multiple
 *            i2c components to be connected to one component.  
 *
 * @param dev I2C address of the desired IC. Needs to be even, as the lowest bit
 *            of the address is a read/write flag on the bus. If only 7 bits are
 *            provided by the datasheet, shift it by one position to the left.
 * 
 * @param reg Register offset byte send before buf. If -1 will be ignored.
 * 
 * @param len Number of bytes to be written from buf.
 * 
 * @param written pointer that returns how many bytes were actually written to
 *                the Bus
 * 
 * @param buf Pointer to buffer containing the bytes to be written.
 * 
 * @return Implementation sepcific.
 * 
 */
I2C_Error_t i2c_write_reg(const if_I2C_t* bus,
                          int dev,
                          int reg,
                          size_t len,
                          size_t *written,
                          const uint8_t* buf);

/**
 * @brief Read number of bytes from I2C Bus
 * 
 * @param bus Pointer to the if_i2c management struct, that allows multiple
 *            i2c components to be connected to one component.  
 *
 * @param dev I2C address of the desired IC. Needs to be even, as the lowest bit
 *            of the address is a read/write flag on the bus. If only 7 bits are
 *            provided by the datasheet, shift it by one position to the left.
 * 
 * @param len Number of bytes to be written from buf.
 * 
 * @param read  pointer that returns how many bytes were actually read from
 *                the Bus
 * 
 * @param buf Pointer to buffer containing the bytes to be written.
 * 
 * @return Implementation sepcific.
 */
I2C_Error_t i2c_read(const if_I2C_t* bus,
                     int dev,
                     size_t len,
                     size_t *read,
                     uint8_t* buf);