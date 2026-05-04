/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * bmp388.h - BMP388 barometric pressure sensor driver header
 * 
 * Copyright (C) 2025-2026 Ahmet Sekercioglu and Ismet Atalar
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef BMP388_H
#define BMP388_H

#include <stdint.h>
#include <math.h>
#include <inttypes.h> // Include for standard fixed-width format specifiers (e.g., PRIu32)
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define BMP388_CHIP_ID_REG        0x00  // Register to read BMP388 chip ID
#define BMP388_CHIP_ID            0x50  // Expected chip ID for BMP388
#define BMP388_TEMP_REG           0x22  // Temperature register (LSB)
#define BMP388_PRESSURE_REG       0x1F  // Pressure register (LSB)
#define BMP388_CALIB_ADDR         0x31
#define BMP388_CMD_REG            0x7E
#define BMP388_DATA_ADDR          0x04
#define BMP388_PWR_CTRL_REG       0x1B  // Power control register
#define BMP388_OSR_REG            0x1C  // OSR Over Sampling Register
#define BMP388_ODR_REG            0x1D  // ODR Output Data Rate Register
#define BMP388_CONFIG_REG         0x1F  // IIR filter Coefficients Register

typedef struct {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;  // Linearized temperature used for pressure compensation
} bmp388_calib_data_t;

// Function prototypes
void initialize_bmp388(i2c_master_dev_handle_t bmp388_dev_handle, int i2c_master_timeout_ms);
void bmp388_read_coefficients(i2c_master_dev_handle_t dev_handle, bmp388_calib_data_t *calib_data, int i2c_master_timeout_ms);
float bmp388_compensate_temperature(uint32_t uncomp_temp, bmp388_calib_data_t *calib_data);
float bmp388_compensate_pressure(uint32_t uncomp_press, bmp388_calib_data_t *calib_data);
void bmp388_read_data(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms);
void read_bmp388_chip_id(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms);

#endif // BMP388_H
