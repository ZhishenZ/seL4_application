#include "ssd1306.h"
#include "fonts.h"
#include <camkes.h>
#include <stdbool.h>
#include "i2c_lib.h"
#include "lib_debug/Debug.h"

#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH 128
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT 32
#endif

static ssd1306_t ssd1306_dev = {
    .protocol = SSD1306_PROTO_I2C,
    .screen = SSD1306_SCREEN,
    .i2c_dev.bus = IF_I2C_ASSIGN(i2c_rpc, i2c_port, i2cBus_notify),
    .i2c_dev.addr = SSD1306_I2C_ADDR_0,
    .width = DISPLAY_WIDTH,
    .height = DISPLAY_HEIGHT
};


OS_Dataport_t port_storage = OS_DATAPORT_ASSIGN(ssd1306_port);
static uint8_t buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];

static bool init = false;

void post_init(void)
{
    Debug_LOG_DEBUG("[%s] %s running", get_instance_name(), __func__);
    init = false;

    i2c_wait_for_bus_initialisation(&(ssd1306_dev.i2c_dev.bus));

    i2c_init_slave(&(ssd1306_dev.i2c_dev.bus), (ssd1306_dev.i2c_dev.addr << 1) );
 
    int ret = ssd1306_init(&ssd1306_dev);
    if(ret != 0)
    {
        Debug_LOG_ERROR("[%s] ssd1306 failed to initialise, aborting", get_instance_name());
        return;
    }
    ssd1306_set_whole_display_lighting(&ssd1306_dev, false);
    init = true;
    Debug_LOG_INFO("[%s] ssd1306 initialised", get_instance_name());
}

void ssd1306_rpc_clear_display(void)
{
    if(!init)
    {
        Debug_LOG_ERROR("ssd1306 not initialised");
        return;
    }
    int ret = ssd1306_fill_rectangle(&ssd1306_dev, buffer, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, OLED_COLOR_BLACK);
    if(ret != 0)
    {
        Debug_LOG_ERROR("ssd1306_fill_rectangle() returned %d", ret);
    }
    ssd1306_load_frame_buffer(&ssd1306_dev, buffer);
}

int ssd1306_rpc_write_string(int len, int font_face, uint8_t x, uint8_t y)
{
    if(!init)
    {
        Debug_LOG_ERROR("ssd1306 not initialised");
        return -1;
    }
    size_t dataport_size = OS_Dataport_getSize(port_storage);
    if(len + 1 > dataport_size)
    {
        // the client did a bogus request, it knows the data port size and
        // never ask for more data
        Debug_LOG_ERROR(
            "ssd1306_rpc_write() size %zu exceeds dataport size %zu",
            len,
            dataport_size
        );
        return -1;
    }
    char * str = (char *) OS_Dataport_getBuf(port_storage);
    *(str + len) = '\0';
    int ret_str = ssd1306_draw_string(&ssd1306_dev, buffer, font_builtin_fonts[font_face], x, y, str,OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    if(ret_str < 0)
    {
        Debug_LOG_ERROR("ssd1306_draw_string() returned %d", ret_str);
    }

    return ret_str;
}

int ssd1306_rpc_update_display(void)
{
    int ret = ssd1306_load_frame_buffer(&ssd1306_dev, buffer);
    if(ret != 0)
    {
        Debug_LOG_ERROR("ssd1306_load_frame_buffer() returned %d", ret);
        return -1;
    }
    return ret;
}