

#include "bmp280.h"
#include <camkes.h>
#include <stdbool.h>

#include "i2c_lib.h"

#include "lib_debug/Debug.h"

static bmp280_t bmp280_dev = {
    .i2c_dev.bus = IF_I2C_ASSIGN(i2c_rpc, i2c_port, i2cBus_notify),
    .i2c_dev.addr = BMP280_I2C_ADDRESS_0,
};

static bool init = false;

/**
 * @brief Function called when module is started
 * 
 * @details Initialises the BMP280 library used. Once ready sets internal init
 *          state to true. Sensor library will read out some calibration
 *          Data from the sensor, so this can take some time.
 * 
 */
void post_init(void)
{
    Debug_LOG_DEBUG("[%s] %s running", get_instance_name(), __func__);
    init = false;
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    i2c_wait_for_bus_initialisation(&(bmp280_dev.i2c_dev.bus));
    
    Debug_LOG_INFO("[%s] I2C ready, now initialising sensor", get_instance_name());
    i2c_init_slave(&(bmp280_dev.i2c_dev.bus), (bmp280_dev.i2c_dev.addr << 1) );
    
    bmp280_init(&bmp280_dev, &params);
    Debug_LOG_INFO("bmp280 initialised");
    init = true;
}

void bmp280_rpc_get_data(float* temperature, float* pressure, float* humidity)
{
    if(!init)
    {
        *temperature = 0.0;
        *pressure = 0.0;
        *humidity = 0.0;
        return;
    }
    bmp280_read_float(&bmp280_dev, temperature, pressure, humidity);

    bool bme280p = bmp280_dev.id == BME280_CHIP_ID;
    if(!bme280p)
    {
        *humidity = 0.0;
    }
}

bool bmp280_rpc_sensor_ready(void)
{
    return init;
}
