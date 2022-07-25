
# BMP280 Component

This Component connects ofer the I2C CAmkES interface and I2C hardware connection to an BMP280 sensor.
The sensor is capable of measuring air temperature, air pressure, and air humidity.
The component uses a modified version of the bmp280 driver in [https://github.com/SuperHouse/esp-open-rtos.git](https://github.com/SuperHouse/esp-open-rtos.git) .

## RPC interface

The component provides one rpc function called get_data(). It returns temperature, pressure, and humidity as floats via a pointer.
The setup of the bmp280 is handled by the component in its post_init() function.

## How to use

In the main camkess file add the following:

```
...
#include "components/bmp280/bmp280.camkes"


...
BMP280_COMPONENT_DEFINE(BMP280)
...
component "bar app" {
    ...
    uses    if_BMP280   bmp280_rpc;
    ...
}
...
assembly {
    composition{
        ...
        component BMP280 bmp280;
        ...

        I2CI_INSTANCE_CONNECT_CLIENT(
            "foo i2c",
            bmp280.i2c_rpc, bmp280.i2c_port,
            bmp280.i2cBus_notify, "number"
        )
        connection seL4RPCCall bmp280_rpc (
          from "bar app".bmp280_rpc,
          to bmp280.bmp280_rpc
        );
        ...
    }
    ...
}
```

Please replace without ":

* "foo i2c" with the actual name of the I2C component
* "number" with an incremented number on the I2C Interface. See its documentation for more details
* "bar app" with the components name that actually uses the sensor and needs the data.

Inside the source of "bar app" add

```C
float temperature, pressure, humidity;
bmp280_rpc_get_data(&temperature, &pressure, &humidity);

```

## Modification made to library

The bmp280 library was taken from [https://github.com/SuperHouse/esp-open-rtos.git](https://github.com/SuperHouse/esp-open-rtos.git).
You can find it under extras/bmp280 inside this repository.
In this component the files are located inside the 3rdParty directory.
They were only slightly edited to use the i2c CAmkES interface instead of the original i2C interface.
Simmilar approaches can be used for other i2c libraries to get components to work.
This Component can be used as refference.
