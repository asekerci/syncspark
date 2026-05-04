/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * drv5033.h - DRV5033 Hall effect sensor driver for wheel turn counting
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

// Hall Effect Sensors for Wheel Turn Counting
#ifndef DRV5033_H
#define DRV5033_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdbool.h>

// Sensor selection enum
typedef enum {
    SENSOR_1 = 0,  // Left wheel sensor
    SENSOR_2 = 1   // Right wheel sensor
} hall_sensor_id_t;

// Default configuration values
#define HALL_SENSOR_DEFAULT_MAGNETS_PER_REV    4
#define HALL_SENSOR_DEFAULT_INVERT_LOGIC       false

// Hall effect sensor configuration structure (for a single sensor)
typedef struct {
    gpio_num_t sensor_gpio;         // GPIO pin for the sensor
    uint8_t magnets_per_revolution; // Number of magnets on the magnet disk (default: 4)
    bool invert_logic;              // If true, magnet presence = HIGH, absence = LOW
} hall_sensor_config_t;

// Hall sensor data structure (for a single sensor)
typedef struct {
    volatile uint32_t sensor_count;     // Total pulses from sensor
    volatile int64_t last_sensor_time;  // Last sensor trigger time (us)
    volatile bool sensor_state;         // Current sensor state
    hall_sensor_config_t sensor_config; // Sensor configuration
} hall_sensor_data_t;

// Convenience macro for default configuration of a single sensor
#define HALL_SENSOR_DEFAULT_CONFIG(sensor_pin) { \
    .sensor_gpio = sensor_pin, \
    .magnets_per_revolution = HALL_SENSOR_DEFAULT_MAGNETS_PER_REV, \
    .invert_logic = HALL_SENSOR_DEFAULT_INVERT_LOGIC \
}

// Function prototypes
esp_err_t initialize_hall_sensors(const hall_sensor_config_t *sensor_1_config, 
                                  const hall_sensor_config_t *sensor_2_config);
esp_err_t deinitialize_hall_sensors(void);
uint32_t get_sensor_count(hall_sensor_id_t sensor_id); // Get count for specified sensor
int64_t get_time_since_last_pulse_us(hall_sensor_id_t sensor_id); // Time in microseconds, -1 if no pulse detected yet
void reset_sensor_counts(void);
float calculate_wheel_revolutions(hall_sensor_id_t sensor_id);
float calculate_wheel_angle_degrees(hall_sensor_id_t sensor_id);
float calculate_wheel_rpm(hall_sensor_id_t sensor_id, uint32_t time_window_ms);

/**
 * @brief Get current sensor states (for debugging)
 * 
 * @param sensor_1_state Pointer to store current sensor_1 sensor state
 * @param sensor_2_state Pointer to store current sensor_2 sensor state
 */
void get_sensor_states(bool *sensor_1_state, bool *sensor_2_state);

/**
 * @brief Wait for next sensor_2 sensor pulse (blocking)
 * 
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return true if sensor_2 pulse detected, false if timeout
 */
bool wait_for_sensor_2_pulse(uint32_t timeout_ms);

/**
 * @brief Check if wheel is currently moving based on recent activity
 * 
 * @param sensor_id Which sensor/wheel to check
 * @param threshold_ms Time threshold in milliseconds (if no pulse within this time, consider stopped)
 * @return true if wheel appears to be moving, false if stopped
 */
bool is_wheel_moving(hall_sensor_id_t sensor_id, uint32_t threshold_ms);

#endif // DRV5033_H
