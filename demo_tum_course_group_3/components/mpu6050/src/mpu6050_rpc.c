#include "mpu6050.h"
#include <camkes.h>
#include <stdbool.h>

#include "i2c_lib.h"

#include "lib_debug/Debug.h"

static mpu6050_t mpu6050_dev = {
    .i2c_dev.bus = IF_I2C_ASSIGN(i2c_rpc, i2c_port, i2cBus_notify),
    .i2c_dev.addr = MPU6050_I2C_ADDRESS_0,
    .id = MPU6050_CHIP_ID,
};

static bool init = false;

/**
 * @brief Function called when module is started
 * 
 * @details Initialises the MPU6050 library used. Once ready sets internal init
 *          state to true.
 * 
 */
void post_init(void)
{
    Debug_LOG_DEBUG("[%s] %s running", get_instance_name(), __func__);
    init = false;
    mpu6050_params_t params;
    mpu6050_init_default_params(&params);
    i2c_wait_for_bus_initialisation(&(mpu6050_dev.i2c_dev.bus));
    
    Debug_LOG_INFO("[%s] I2C ready, now initialising sensor", get_instance_name());
    i2c_init_slave(&(mpu6050_dev.i2c_dev.bus), (mpu6050_dev.i2c_dev.addr << 1) );
    
    if(mpu6050_init(&mpu6050_dev, &params))
    {
        Debug_LOG_INFO("mpu6050 initialised");
        init = true;
    }

}

void mpu6050_rpc_get_data(uint16_t *accelX, uint16_t *accelY, uint16_t *accelZ, uint16_t *gyroX, uint16_t *gyroY, uint16_t *gyroZ)
{
    if(!init)
    {
        *accelX = 0;
        *accelY = 0;
        *accelZ = 0;
        *gyroX = 0;
        *gyroY = 0; 
        *gyroZ = 0;
        return;
    }
    mpu6050_read(&mpu6050_dev, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
}

bool mpu6050_rpc_sensor_ready(void)
{
    return init;
}