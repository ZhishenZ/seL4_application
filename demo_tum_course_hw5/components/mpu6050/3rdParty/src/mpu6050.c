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

#define MPU6050_RA_SIGNAL_PATH_RESET 0x68

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


void mpu6050_init_default_params(mpu6050_params_t *params)
{
    params->mode = MPU6050_MODE_WAKEUP;
}


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

    if (dev->i2c_dev.addr != MPU6050_I2C_ADDRESS_0 && dev->i2c_dev.addr != MPU6050_I2C_ADDRESS_1) {
        debug("Invalid I2C address");
        return false;
    }


    // if (read_data(&dev->i2c_dev, BMP280_REG_ID, &dev->id, 1)) {
    //     debug("Sensor not found");
    //     return false;
    // }

    uint8_t q;
    read_data(&dev->i2c_dev, 0x75, &q, 1);
    debug("WHO_AM_I: %x, %d", q, q);


    if (dev->id != MPU6050_CHIP_ID) {
        debug("Sensor wrong version");
        return false;
    }

    // // Soft reset.
    // if (write_register8(&dev->i2c_dev, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
    //     debug("Failed resetting sensor");
    //     return false;
    // }

    // // Wait until finished copying over the NVP data.
    // while (1) {
    //     uint8_t status;
    //     if (!read_data(&dev->i2c_dev, BMP280_REG_STATUS, &status, 1) && (status & 1) == 0)
    //         break;
    // }

    // if (!read_calibration_data(dev)) {
    //     debug("Failed to read calibration data");
    //     return false;
    // }

    // if (dev->id == BME280_CHIP_ID && !read_hum_calibration_data(dev)) {
    //     debug("Failed to read humidity calibration data");
    //     return false;
    // }

    // uint8_t config = (params->standby << 5) | (params->filter << 2);
    debug("Writing 0x6B reg=%x", params->mode);
    if (write_register8(&dev->i2c_dev, MPU6050_RA_PWR_MGMT_1, params->mode)) {
        debug("Failed to wake up the sensor!");
        return false;
    }

    // if (params->mode == BMP280_MODE_FORCED) {
    //     params->mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
    // }

    // uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2)
    //     | (params->mode);


    // if (dev->id == BME280_CHIP_ID) {
    //     // Write crtl hum reg first, only active after write to BMP280_REG_CTRL.
    //     uint8_t ctrl_hum = params->oversampling_humidity;
    //     debug("Writing ctrl hum reg=%x", ctrl_hum);
    //     if (write_register8(&dev->i2c_dev, BMP280_REG_CTRL_HUM, ctrl_hum)) {
    //         debug("Failed controlling sensor");
    //         return false;
    //     }
    // }

    // debug("Writing ctrl reg=%x", ctrl);
    // if (write_register8(&dev->i2c_dev, BMP280_REG_CTRL, ctrl)) {
    //     debug("Failed controlling sensor");
    //     return false;
    // }

    return true;
}

void mpu6050_read(mpu6050_t *dev, uint16_t *accelX, uint16_t *accelY, uint16_t *accelZ, uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ)
{
    // uint8_t accelX_high, accelX_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_XOUT_H, & accelX_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_XOUT_L, & accelX_low, 1);
    // *accelX = accelX_high*256 + accelX_low;
    // debug("AccelX=%d", *accelX);

    // uint8_t accelY_high, accelY_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_YOUT_H, & accelY_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_YOUT_L, & accelY_low, 1);
    // *accelY = accelY_high*256 + accelY_low;
    // debug("AccelX=%d", *accelY);

    // uint8_t accelZ_high, accelZ_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_ZOUT_H, & accelZ_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_ACCEL_ZOUT_L, & accelZ_low, 1);
    // *accelZ = accelZ_high*256 + accelZ_low;
    // debug("AccelX=%d", *accelZ);

    // uint8_t gyroX_high, gyroX_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_XOUT_H, & gyroX_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_XOUT_L, & gyroX_low, 1);
    // *gyroX = gyroX_high*256 + gyroX_low;
    // debug("GyroX=%d", *gyroX);

    // uint8_t gyroY_high, gyroY_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_YOUT_H, & gyroY_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_YOUT_L, & gyroY_low, 1);
    // *gyroY = gyroY_high*256 + gyroY_low;
    // debug("GyroY=%d", *gyroY);

    // uint8_t gyroZ_high, gyroZ_low;
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_ZOUT_H, & gyroZ_high, 1);
    // read_data(&dev->i2c_dev, MPU6050_RA_GYRO_ZOUT_L, & gyroZ_low, 1);
    // *gyroZ = gyroZ_high*256 + gyroZ_low;
    // debug("GyroZ=%d", *gyroZ);

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