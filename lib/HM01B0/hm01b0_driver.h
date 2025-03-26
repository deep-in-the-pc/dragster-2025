//
// SPDX-FileCopyrightText: Copyright 2023 Arm Limited and/or its affiliates <open-source-office@arm.com>
// SPDX-License-Identifier: MIT
//

#ifndef _PICO_HM01B0_H_
#define _PICO_HM01B0_H_

#include "hardware/i2c.h"
#include "hardware/pio.h"

struct hm01b0_config {
    i2c_inst_t* i2c;
    uint sda_pin;
    uint scl_pin;

    uint vsync_pin;
    uint hsync_pin;
    uint pclk_pin;

    uint data_pin_base;
    uint data_bits;
    PIO pio;
    uint pio_sm;

    int reset_pin;
    int mclk_pin;

    uint width;
    uint height;
};

int hm01b0_init(const struct hm01b0_config* config);
void hm01b0_deinit();

void hm01b0_read_frame(uint8_t* buffer, size_t length);
void hm01b0_set_coarse_integration(unsigned int lines);

/* --- New functions for runtime configuration --- */

/**
 * @brief Reconfigure the sensor data interface.
 *
 * This function sets the sensor to output image data using the specified number of data bits.
 * Valid values are 8, 4, or 1. For example, setting data_bits to 4 will configure the sensor
 * to output data on D0-D3.
 *
 * @param data_bits The desired number of data bits (8, 4, or 1).
 * @return 0 on success, -1 on error.
 */
int hm01b0_set_data_interface(uint8_t data_bits);

/**
 * @brief Change the sensor output resolution.
 *
 * This function updates various sensor registers to configure the output window and timing
 * according to the requested resolution. Supported resolutions are:
 *   - 320x320
 *   - 320x240
 *   - 160x120
 *
 * @param width  The desired output width in pixels.
 * @param height The desired output height in pixels.
 * @return 0 on success, -1 on invalid resolution.
 */
int hm01b0_set_resolution(uint16_t width, uint16_t height);

#endif
