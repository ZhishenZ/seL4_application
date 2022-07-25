

#include "lib_debug/Debug.h"

#include "plat_i2c.h"
#include <camkes.h>
#include <camkes/io.h>
#include "imx6_i2c.h"

typedef struct i2c_config {
    enum i2c_id id;
    enum i2c_slave_speed slave_speed;
    enum i2c_slave_address_size addr_size;
} i2c_config_t;

static i2c_bus_t        i2c_bus;
static ps_io_ops_t      io_ops;
i2c_slave_t             slave[MAX_I2C_SLAVES];
volatile static int              setup_slaves;                     
i2c_config_t            config;

bool _i2c_init(void)
{
    Debug_LOG_DEBUG("[%s] %s", get_instance_name(), __func__);

    setup_slaves = 0;

    config.id = I2C_CONFIG_ID;
    config.slave_speed = I2C_SLAVE_SPEED_STANDARD;
    config.addr_size = I2C_SLAVE_ADDR_7BIT;

    int error = camkes_io_ops(&io_ops);
    if (error)
    {
        Debug_LOG_ERROR("camkes_io_ops() failed: rslt = %i", error);
        return false;
    }

    error = mux_sys_init(&io_ops,NULL,&io_ops.mux_sys);
    if (error)
    {
        Debug_LOG_ERROR("mux_sys_init() failed: rslt = %i", error);
        return false;
    }

    error = clock_sys_init(&io_ops,&io_ops.clock_sys);
    if (error)
    {
        Debug_LOG_ERROR("clock_sys_init() failed: rslt = %i", error);
        return false;
    }

    error = i2c_init(config.id,&io_ops,&i2c_bus);
    if(error){
        Debug_LOG_ERROR("i2c_init() failed: rslt = %i", error);
        return false;
    }

    Debug_LOG_INFO("%s done",__func__);
    return true;

}

int find_slave(size_t dev)
{
    for(int i = 0; i < setup_slaves; i++)
    {
        if(slave[i].address == (dev >> 1) )
        {
            //Debug_LOG_INFO("Found I2C Device %zx", dev);
            return i;
        }
    }
    return -1;
}

I2C_Error_t 
__attribute__((__nonnull__))
 _i2c_init_slave(
    int dev )
{

    if(find_slave(dev) != -1)
    {
        Debug_LOG_WARNING("I2C slave with address %zx already setup", dev);
        return I2C_SUCCESS;
    }
    if(setup_slaves >= MAX_I2C_SLAVES)
    {
        Debug_LOG_ERROR("Maximum number of allowed slaves reached");
        return I2C_ERROR_GENERIC;
    }
    Debug_LOG_DEBUG("Slave initialization of device@0x%zx.",dev);
    int ret = i2c_bus.slave_init(&i2c_bus,
                              dev,
                              config.addr_size,
                              config.slave_speed,
                              0,
                              &slave[setup_slaves]);
    if(ret){
        Debug_LOG_ERROR("i2c slave_init failed, return code: %d, ic: %zx",ret, dev);
        return I2C_ERROR_GENERIC;
    }
    setup_slaves++;
    Debug_LOG_DEBUG("Slave initialization for %zx done.",dev);
    return I2C_SUCCESS;
}


I2C_Error_t _i2c_write(int dev, const uint8_t* buf, size_t len, size_t* written)
{
    *written = 0;
    int needed_slave = find_slave(dev);
    if(needed_slave == -1)
    {
        Debug_LOG_ERROR("Slave %zx not initialiseed", dev);
        return I2C_ERROR_GENERIC;
    }

    int ret = slave[needed_slave].slave_write(&slave[needed_slave],
                                            buf,
                                            len,
                                            false,
                                            NULL,
                                            NULL);
    if(ret < 0)
    {
        Debug_LOG_ERROR("slave_write() returend error %d", ret);
        return I2C_ERROR_GENERIC;
    }
    *written = ret;
    if(ret != len)
    {
        Debug_LOG_ERROR("Writing to i2c slave device failed.");
        return I2C_ERROR_TRY_AGAIN;
    }
    return I2C_SUCCESS;
}

I2C_Error_t _i2c_read(int dev, uint8_t* buf, size_t len, size_t* read)
{
    *read = 0;

    int needed_slave = find_slave(dev);
    if(needed_slave == -1)
    {
        Debug_LOG_ERROR("Slave %zx not initialiseed", dev);
        return I2C_ERROR_GENERIC;
    }

    int ret = slave[needed_slave].slave_read(&slave[needed_slave],
                                            buf,
                                            len,
                                            false,
                                            NULL,
                                            NULL);

    if(ret < 0)
    {
        Debug_LOG_ERROR("slave_read() returend error %d", ret);
        return I2C_ERROR_GENERIC;
    }
    *read = ret;
    if(ret != len)
    {
        Debug_LOG_ERROR("Reading  to i2c slave device failed.");
        return I2C_ERROR_TRY_AGAIN;
    }
    return I2C_SUCCESS;
}