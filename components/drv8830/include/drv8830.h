/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * drv8830.h - DRV8830 motor driver header for SynchroSpark
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

#ifndef DRV8830_H
#define DRV8830_H

#include <inttypes.h> // Include for standard fixed-width format specifiers 
                      // (e.g., PRIu32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// Turn direction for differential drive
typedef enum {
    TURN_LEFT = 0,
    TURN_RIGHT
} robot_turn_direction_t;

// Optional stop behavior for DRV8830
typedef enum {
    DRV8830_STOP_COAST = 0x00, // IN1:IN2 = 00
    DRV8830_STOP_BRAKE = 0x03  // IN1:IN2 = 11
} drv8830_stop_mode_t;

// Function prototypes -- project-level global functions
void initialize_motor(i2c_master_dev_handle_t dev_handle, int);
void stop_motor(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms);
void drv8830_set_motor(i2c_master_dev_handle_t dev_handle, uint8_t speed, bool direction, int i2c_master_timeout_ms);



// Enhanced controls
void stop_motor_with_mode(i2c_master_dev_handle_t dev_handle, drv8830_stop_mode_t mode, int i2c_master_timeout_ms);
esp_err_t drv8830_read_fault(i2c_master_dev_handle_t dev_handle, uint8_t *fault_reg_out, int i2c_master_timeout_ms);

// Robot control functions using motor configuration
// motor_config should point to a sparknode_motor_config_t structure
esp_err_t drive_robot(i2c_master_dev_handle_t left_motor,
                      i2c_master_dev_handle_t right_motor,
                      uint8_t left_speed,
                      uint8_t right_speed,
                      bool forward,
                      uint32_t duration_milliseconds,
                      const void* motor_config);

esp_err_t turn_robot(i2c_master_dev_handle_t left_motor,
                     i2c_master_dev_handle_t right_motor,
                     robot_turn_direction_t direction,
                     uint8_t left_speed,
                     uint8_t right_speed,
                     uint32_t duration_milliseconds,
                     const void* motor_config);

// Camera motor helpers
// Jog the camera motor gently for a short duration with ramped start/stop.
// - speed: 0..0x3F (suggest 0x06..0x12 for fine motion)
// - inward: app-defined direction (true/false)
// - duration_ms: time at target speed (excludes ramp time)
void camera_motor_jog(i2c_master_dev_handle_t cam_motor,
                      uint8_t speed,
                      bool inward,
                      uint32_t duration_ms,
                      int i2c_master_timeout_ms);

#endif // DRV8830_H
