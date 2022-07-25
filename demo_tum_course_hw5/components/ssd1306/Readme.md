# SSD1306 component

This IC is a display driver for small OLED displays.

It currently can be used to write Text to the display. However the library used in the background can handle more.

## How to use

inside the main camkes file:

```
...
#include "components/ssd1306/ssd1306.camkes"
...
component <foo> {
    ...
    uses        if_SSD1306          <ssd1306_rpc>;
    dataport    Buf                 <ssd1306_port>;
    ...
}
...
assembly{
    composition{
        ...
        component SSD1306 <ssd1306>;
        ...
        connection seL4RPCCall ssd1306_rpc (
          from <foo>.<ssd1306_rpc>,
          to <ssd1306>.ssd1306_rpc
        );
        connection seL4SharedData ssd1306_port (
          from <foo>.<ssd1306_port>,
          to <ssd1306>.ssd1306_port
        );
        ...
    }
    ...
}
```

Inside the cmake files:
```
...
include(${CMAKE_CURRENT_SOURCE_DIR}/components/ssd1306/CMakeLists.txt)
SSD1306_Declare_CAmkESComponent(SSD1306)
...
```

Inside the `<foo>` application:

```C
...

#include "ssd1306.h"
#include <string.h>
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT   32

...

OS_Dataport_t ssd1306_port_storage = OS_DATAPORT_ASSIGN(<ssd1306_port>);

int ssd1306_write_string(char * str, int len,font_face_t font_face , uint8_t x, uint8_t y)
{
    uint8_t* ssd1306_buf = OS_Dataport_getBuf(ssd1306_port_storage);
    size_t ssd1306_buf_size = OS_Dataport_getSize(ssd1306_port_storage);
    if(*(str  + len) != '\0')
    {
        Debug_LOG_ERROR("C Stings need to terminate with a NULL character");
        return -1;
    }
    if(len + 1 > ssd1306_buf_size)
    {
        Debug_LOG_ERROR("String to big for shared memory");
        return -1;
    }
    memcpy(ssd1306_buf, str, len);
    return <ssd1306_rpc>_write_string(len, (int) font_face, x, y );

}
...
void run(void)
{
    ...
    <ssd1306_rpc>_clear_display();
    char buf[] = "Hello, SeL4!";
    ssd1306_write_string(buf, strlen(buf),FONT_FACE_GLCD5x7, 0, 0);
    <ssd1306_rpc>_update_display();
    ...
}
...

```

## External Library used

The ssd1306 library was taken from [https://github.com/SuperHouse/esp-open-rtos.git](https://github.com/SuperHouse/esp-open-rtos.git).
You can find it under extras/ssd1306 and extras/fonts inside this repository.
In this component the files are located inside the 3rdParty directory.
They were only slightly edited to use the i2c CAmkES interface instead of the original i2C interface.
Simmilar approaches can be used for other i2c libraries to get components to work.
This Component can be used as refference.

