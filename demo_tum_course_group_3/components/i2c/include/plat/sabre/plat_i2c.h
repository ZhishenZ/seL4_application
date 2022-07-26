
#pragma once

#include "plat_i2c_generic.h"

#include "arch/arm/i2c.h"

/**
 * @brief Configuration option for internal slave struct array. 
 *        Increase if more than 20 Devices are needed on the Bus.
 */
#define MAX_I2C_SLAVES 20