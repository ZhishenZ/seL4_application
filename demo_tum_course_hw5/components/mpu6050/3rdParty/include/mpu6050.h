#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <stdbool.h>
#include "i2c_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Uncomment to enable debug output.
 */
// #define MPU6050_DEBUG

/**
 * MPU6050 address is 0x69 if SDO pin is high, and is 0x68 (default) if
 * SDO pin is low.
 */
#define MPU6050_I2C_ADDRESS_0  0x68
#define MPU6050_I2C_ADDRESS_1  0x69

#define MPU6050_CHIP_ID  0x68 /* MPU6050 has chip-id 0x68 */

typedef enum {
    MPU6050_MODE_SLEEP = 1,
    MPU6050_MODE_WAKEUP = 0
} MPU6050_Mode;


typedef struct {
    MPU6050_Mode mode;
} mpu6050_params_t;


typedef struct
{
  if_I2C_t bus;
  uint8_t addr;
} i2c_dev_t;


typedef struct {
    i2c_dev_t  i2c_dev;  /* I2C dev setting. */
    uint8_t  id;        /* Chip ID */
} mpu6050_t;


/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: SLEEP
 */
void mpu6050_init_default_params(mpu6050_params_t *params);


/**
 * Initialize MPU6050 module, probes for the device, soft resets the device,
 * reads the calibration constants, and configures the device using the supplied
 * parameters. Returns true on success otherwise false.
 *
 * The I2C address is assumed to have been initialized in the dev, and
 * may be either MPU6050_I2C_ADDRESS_0 or MPU6050_I2C_ADDRESS_1. If the I2C
 * address is unknown then try initializing each in turn.
 *
 * This may be called again to soft reset the device and initialize it again.
 */
bool mpu6050_init(mpu6050_t *dev, mpu6050_params_t *params);

/**
 * Read compensated temperature and pressure data:
 *  Temperature in degrees Celsius.
 *  Pressure in Pascals.
 *  Humidity is optional and only read for the BME280, in percent relative
 *  humidity.
 */

void mpu6050_read(mpu6050_t *dev, uint16_t *accelX, uint16_t *accelY, uint16_t *accelZ, uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ);



#ifdef __cplusplus
}
#endif

#endif  // __MPU6050_H__