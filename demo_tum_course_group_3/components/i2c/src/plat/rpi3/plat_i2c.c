
#include "lib_debug/Debug.h"

#include "bcm2837_gpio.h"
#include "bcm2837_i2c.h"
#include "plat/rpi3/plat_i2c.h"



#include <camkes.h>

void print_gpio_regs(void);
void print_i2c_regs(void);

bool _i2c_init(void)
{
    // Initialise Support library
    if(!bcm2837_i2c_begin(regBase, gpioBase))
    {
        Debug_LOG_ERROR("bcm2837_i2c_begin() failed");
        return false;
    }
    bcm2837_i2c_set_baudrate(100000);
    // Print Peripheral registers
    print_gpio_regs();
    print_i2c_regs();
    return true;
}

I2C_Error_t _i2c_init_slave(int dev)
{
    return I2C_ERROR_NOT_IMPLEMENTED;
}

I2C_Error_t _i2c_write(int dev, const uint8_t* buf, size_t len, size_t* written)
{
    int ret = 0;
    *written = 0;

    bcm2837_i2c_setSlaveAddress((uint8_t) (dev >> 1));

    ret = bcm2837_i2c_write((char*) buf, len);

    if(ret == BCM2837_I2C_REASON_ERROR_NACK)
    {
        Debug_LOG_INFO("bcm2837_i2c_write() returned NACK error");
        return I2C_ERROR_NACK;
    }
    if(ret == BCM2837_I2C_REASON_ERROR_CLKT)
    {
        Debug_LOG_INFO("bcm2837_i2c_write() returned clock stretching timeout");
        return I2C_ERROR_CLK_TIMEOUT;        
    }
    if(ret < 0)
    {
        Debug_LOG_WARNING("bcm2837_i2c_write() returned Unsupported error %d", ret);
        return I2C_ERROR_GENERIC;      
    }
    *written = ret;
    if(ret != len)
    {
        Debug_LOG_INFO("bcm2837_i2c_write() write transaction not completed");
        return I2C_ERROR_TRY_AGAIN;
    }
    return I2C_SUCCESS;
}


I2C_Error_t _i2c_read(int dev, uint8_t *buf, size_t len, size_t *read)
{
    int ret = 0;
    *read = 0;

    bcm2837_i2c_setSlaveAddress((uint8_t) (dev >> 1));

    ret = bcm2837_i2c_read((char*) buf, len);

    if(ret == BCM2837_I2C_REASON_ERROR_NACK)
    {
        Debug_LOG_INFO("bcm2837_i2c_read() returned NACK error");
        return I2C_ERROR_NACK;
    }
    if(ret == BCM2837_I2C_REASON_ERROR_CLKT)
    {
        Debug_LOG_INFO("bcm2837_i2c_read() returned clock stretching timeout");
        return I2C_ERROR_CLK_TIMEOUT;        
    }
    if(ret < 0)
    {
        Debug_LOG_WARNING("bcm2837_i2c_read() returned Unsupported error %d", ret);
        return I2C_ERROR_GENERIC;      
    }
    *read = ret;
    if( len != ret)
    {
        Debug_LOG_INFO("bcm2837_i2c_read() read transaction not completed %i %i", ret, len);
        return I2C_ERROR_TRY_AGAIN;
    }
    return I2C_SUCCESS;
}


//----------------------------------------------------
// Debug Functions
//----------------------------------------------------

void print_i2c_regs(void)
{
    volatile uint32_t *bcm2837_bsc = (uint32_t*)regBase;
    
    Debug_LOG_DEBUG("Base address is %p", bcm2837_bsc);
    Debug_LOG_DEBUG("Controll register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("Status register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("dlen register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("Address register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("FIFO register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("div register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("DEL register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
    Debug_LOG_DEBUG("CLKT register at %p : 0x%x, add", bcm2837_bsc, *bcm2837_bsc);
    bcm2837_bsc++;
}


void print_gpio_regs(void)
{
    volatile uint32_t *bcm2837_gpio = (uint32_t*)gpioBase;

    Debug_LOG_DEBUG("Base GPIO address is %p", bcm2837_gpio);

    Debug_LOG_DEBUG("GPFSEL0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFSEL1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFSEL2 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFSEL3 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFSEL4 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFSEL5 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++; // Reserved
    Debug_LOG_DEBUG("GPSET0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPSET10 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++; // Reserved
    Debug_LOG_DEBUG("GPCLR0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPCLR1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPLEV0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPLEV1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPEDS0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPEDS1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPREN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPREN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFEN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPFEN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPHEN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPHEN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPLEN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPLEN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPAREN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPAREN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPAFEN0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPAFEN1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPPUD register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPPUDCLK0 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
    Debug_LOG_DEBUG("GPPPUDCLK1 register at %p: 0x%x", bcm2837_gpio, *bcm2837_gpio);
    bcm2837_gpio++;
}