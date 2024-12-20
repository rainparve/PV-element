/*
 * Copyright (C) 2023 ACME
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_hdc3020
 *
 * @{
 * @file
 * @brief       Default configuration
 */


#ifndef HDC3020_PARAMS_H
#define HDC3020_PARAMS_H

#include "board.h"
#include "hdc3020.h"
#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default I2C Device (vaikimisi I2C_DEV(0))
 * @{
 */
#ifndef HDC3020_PARAM_I2C_DEV
#define HDC3020_PARAM_I2C_DEV   (I2C_DEV(0)) 
#endif
/**@}*/

/**
 * @name    Set default I2C Address
 * @{
 */
#ifndef HDC3020_PARAM_I2C_ADDR
#define HDC3020_PARAM_I2C_ADDR  (CONFIG_HDC1000_I2C_ADDRESS)
//vaikimisi (CONFIG_HDC1000_I2C_ADDRESS)
#endif
/**@}*/

#ifndef HDC3020_ENABLE_PIN
#define HDC3020_ENABLE_PIN      GPIO_UNDEF
#endif

#ifndef HDC3020_ENABLE_ON
#define HDC3020_ENABLE_ON       (1)
#endif

#ifndef HDC3020_START_DELAY
#define HDC3020_START_DELAY     4000
#endif

#ifndef HDC3020_MEAS_DELAY
#define HDC3020_MEAS_DELAY      22000
#endif

#ifndef HDC3020_PARAMS
#define HDC3020_PARAMS                          \
    {                                           \
        .i2c_dev = HDC3020_PARAM_I2C_DEV,       \
        .i2c_addr = HDC3020_PARAM_I2C_ADDR,     \
        .enable_pin = HDC3020_ENABLE_PIN,       \
        .enable_on = HDC3020_ENABLE_ON,         \
        .start_delay = HDC3020_START_DELAY,     \
        .measure_delay = HDC3020_MEAS_DELAY,    \
    }
#endif

static const hdc3020_params_t hdc3020_params[] =
{
    HDC3020_PARAMS
};

/**
 * @brief   The number of configured sensors
 */
#define HDC3020_NUMOF    ARRAY_SIZE(hdc3020_params)

#ifndef HDC3020_SAULINFO
#define HDC3020_SAULINFO           { .name = "hdc3020" }
#endif

static const saul_reg_info_t hdc3020_saul_info[] =
{
    HDC3020_SAULINFO
};

#ifdef __cplusplus
}
#endif

#endif
