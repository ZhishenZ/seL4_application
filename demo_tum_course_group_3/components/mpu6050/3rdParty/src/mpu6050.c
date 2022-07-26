#include <stddef.h>
#include "mpu6050.h"
#include "lib_debug/Debug.h"


#ifdef MPU6050_DEBUG
//#include <stdio.h>
#define debug(fmt, ...) Debug_LOG_INFO("%s" fmt "\n", "mpu6050: ", ## __VA_ARGS__);
#else
#define debug(fmt, ...)
#endif

/**
 * MPU6050 registers
 */
#define MPU6050_RA_ACCEL_XOUT_H 0x3B
#define MPU6050_RA_ACCEL_XOUT_L 0x3C
#define MPU6050_RA_ACCEL_YOUT_H 0x3D
#define MPU6050_RA_ACCEL_YOUT_L 0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 0x40

#define MPU6050_RA_GYRO_XOUT_H  0x43
#define MPU6050_RA_GYRO_XOUT_L  0x44
#define MPU6050_RA_GYRO_YOUT_H  0x45
#define MPU6050_RA_GYRO_YOUT_L  0x46
#define MPU6050_RA_GYRO_ZOUT_H  0x47
#define MPU6050_RA_GYRO_ZOUT_L  0x48

#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_RA_PWR_MGMT_2 0x6C


//---------------------------------------------------------------------
// Glue Code Modifications, HENSOLDT Cyber
//---------------------------------------------------------------------

int i2c_slave_read(if_I2C_t bus, uint8_t slave_addr, const uint8_t * data, uint8_t *buf, uint32_t len)
{
    size_t tmp = 0;

    int ret = i2c_write(&bus ,(slave_addr << 1), 1, &tmp, data);
    if( ret != I2C_SUCCESS)
    {
        Debug_LOG_ERROR("i2c_write() returned error %d, written were %d bytes", ret, tmp);
        return ret;
    }
    ret = i2c_read(&bus ,(slave_addr << 1), len, &tmp, buf);
    if(ret != I2C_SUCCESS)
    {
       Debug_LOG_ERROR("i2c_read() returned error %d, read were %d bytes", ret, tmp);
       return ret; 
    }
    return ret;
}


static int write_register8(i2c_dev_t *dev, uint8_t addr, uint8_t value)
{
    size_t tmp = 0;
    uint8_t buf[2];
    buf[0] = addr;
    buf[1] = value;

    int ret = i2c_write( &(dev->bus),(dev->addr) << 1, 2, &tmp, buf);
    if(ret != I2C_SUCCESS)
    {
        Debug_LOG_ERROR("i2c_write() returned %d, written were %d bytes", ret, tmp);
        return ret;
    }
    return ret;
}

//----------------------------------------------------------------------
// End of Glue Code
//----------------------------------------------------------------------

/*
 * Initialize the MPU6050 parameter list.
 */
void mpu6050_init_default_params(mpu6050_params_t *params)
{
    params->mode = MPU6050_MODE_WAKEUP;
}

/*
 * Read the Register per two bytes. 
 * (Input address should be the lower one)
 */
static bool read_register16(i2c_dev_t *dev, uint8_t addr, uint16_t *value)
{
    uint8_t d[] = {0, 0};
    if (!i2c_slave_read(dev->bus, dev->addr, &addr, d, sizeof(d))) {
        *value = d[0] | (d[1] << 8);
        return true;
    }
    return false;
}


static inline int read_data(i2c_dev_t *dev, uint8_t addr, uint8_t *value, uint8_t len)
{
    return i2c_slave_read(dev->bus, dev->addr, &addr, value, len);
}


bool mpu6050_init(mpu6050_t *dev, mpu6050_params_t *params)
{

    /*
     * Check the validation of the i2c address.
     */
    if (dev->i2c_dev.addr != MPU6050_I2C_ADDRESS_0 && dev->i2c_dev.addr != MPU6050_I2C_ADDRESS_1) {
        debug("Invalid I2C address");
        return false;
    }

    /*
     * Get CHIP_ID (The output should be 0x68)
     */
    uint8_t q;
    read_data(&dev->i2c_dev, 0x75, &q, 1);
    debug("WHO_AM_I: %x, %d", q, q);

    /*
     * Check CHIP_ID.
     */
    if (dev->id != MPU6050_CHIP_ID) {
        debug("Sensor wrong version");
        return false;
    }

    /*
     * Wake up the MPU6050.
     */
    debug("Writing 0x6B reg=%x", params->mode);
    if (write_register8(&dev->i2c_dev, MPU6050_RA_PWR_MGMT_1, params->mode)) {
        debug("Failed to wake up the sensor!");
        return false;
    }

    return true;
}


void mpu6050_read(mpu6050_t *dev, uint16_t *accelX, uint16_t *accelY, uint16_t *accelZ, uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ)
{
    read_register16(&dev->i2c_dev, MPU6050_RA_ACCEL_XOUT_L, accelX);
    debug("AccelX=%d", *accelX);
    read_register16(&dev->i2c_dev, MPU6050_RA_ACCEL_YOUT_L, accelY);
    debug("AccelY=%d", *accelY);
    read_register16(&dev->i2c_dev, MPU6050_RA_ACCEL_ZOUT_L, accelZ);
    debug("AccelZ=%d", *accelZ);
    read_register16(&dev->i2c_dev, MPU6050_RA_GYRO_XOUT_L, gyroX);
    debug("GyroX=%d", *gyroX);
    read_register16(&dev->i2c_dev, MPU6050_RA_GYRO_YOUT_L, gyroY);
    debug("GyroY=%d", *gyroY);
    read_register16(&dev->i2c_dev, MPU6050_RA_GYRO_ZOUT_L, gyroZ);
    debug("GyroZ=%d", *gyroZ);
}