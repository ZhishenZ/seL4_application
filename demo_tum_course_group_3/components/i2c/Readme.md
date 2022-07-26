# I2C

This component provides read and write interfaces to I2C peripherals as Master on different platforms.
It currently supports rpi3 and sabre.

## How to Use

For the camkes file the following is required. Everything in <> can be named by the user freely.

```
...
#include "components/i2c/i2c.camkes"
...
#ifdef I2C_HARDWARE_COMPONENTS
I2C_HARDWARE_COMPONENTS(I2CDriverHw)
#endif
...
component <foo> {
    ...
    consumes I2CUnlock <i2cBus_notify>;

    uses    if_I2C      <i2c_rpc>;
    dataport Buf        <i2c_port>;
    ...
}
...
assembly{
    composition{
        ...

        component I2C_<X> <i2cdriver>;
        #ifdef I2C_HARDWARE_COMPONENTS
        component I2CDriverHw <i2cdriverhw>;
        #endif
        ...
        I2C_INSTANCE_CONNECT_CLIENT(
            <i2cdriver>,
            <foo>.<i2c_rpc>, <foo>.<i2c_port>,
            <foo>.<i2cBus_notify> , <num>
        )
        ...
        #ifdef I2C_HARDWARE_COMPONENTS
        I2C_HARDWARE_INSTANCE_CONNECT(
            <i2cdriver>,
            <i2cdriverhw>
        )
        #endif
        ...
    }
    configuration{
        #ifdef I2C_HARDWARE_COMPONENTS
        I2C_HARDWARE_INSTANCE_CONFIGURE_SELF(
            <i2cdriverhw>,
            I2C<X>_ADDR)
        #endif
    }
}
```

* `<foo>` is the application using the i2c interface.
* `<i2c_rpc> <i2c_port> <i2cBus_notify>` are the three neded interface names for connecting the app to the interface
* `<X>` the number of the intended i2c bus. Architecture specific.
* `<i2cdriver>` the name for the i2c i2c interface.
* `<i2cdriverhw>` is required by some architectures.
* `<num>` is an increasing number for all components connected to the i2c interface, starting with 1.

For the Cmake file add the code below. It will instantiate all available and supported i2c on the platform

```Cmake
...
    include(${CMAKE_CURRENT_SOURCE_DIR}/components/i2c/CMakeLists.txt)
    DeclareCAmkESComponents_for_I2Cs()
...
```

Inside the C file, the following is needed to use the i2c_lib functions:

```C
...
#include "i2c_lib.h"
...
if_I2C_t i2c = IF_I2C_ASSIGN(<i2c_rpc>, <i2c_port>, <i2cBus_notify>);

```

`<i2c_rpc> <i2c_port> <i2cBus_notify>` need to be named exactly like in the camkes file prevoiusly.

## Supported Platforms

### RPi3

Needs the I2C_HARDWARE_COMPONENTS

Supports the following I2Cs:

* I2C_1
    * SDA is on GPIO Header Pin 3
    * SCL is on GPIO Header Pin 5
* I2C_0
    * SDA is on GPIO Header Pin 27
    * SCL is on GPIO Header Pin 28

### Sabre

Does not need I2C_HARDWARE_COMPONETS

Supports the following I2Cs:

* I2C_1
* I2C_2
* I2C_3
    * SDA is on Header J7 Pin 5
    * SCL is on Header J7 Pin 6


## 3rdParty Libraries

This device uses the following external libraries:

*  [bcm2835](https://www.airspayce.com/mikem/bcm2835/) for the RPi3 and is Released under MIT by the author. It was only slightly eddited to make it compliant with TrentOS and our Stylguide.
* The SABRE uses the libplatsupport provided by the SeL4 sdk.