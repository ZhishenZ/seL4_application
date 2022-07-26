/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 * Copyright (C) 2021, HENSOLDT Cyber GmbH
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

// Extracted from: sdk/sdk-sel4-camkes/libs/sel4_util_libs/libplatsupport/plat_include/imx6/platsupport/plat/i2c.h

#include <platsupport/io.h>

enum i2c_id {
    I2C1,
    I2C2,
    I2C3,
    NI2C
};

