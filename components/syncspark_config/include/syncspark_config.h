/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * syncspark_config.h - Configuration for SynchroSpark nodes
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

// Configuration file for a SynchroSpark Node
#ifndef SYNCSPARK_CONFIG_H
#define SYNCSPARK_CONFIG_H

#include "driver/i2c_master.h"
#include "led_strip.h"
#include <stdint.h>
#include <stdbool.h>

// Forward declaration to avoid pulling mqtt headers into all components.
struct esp_mqtt_client;
typedef struct esp_mqtt_client *esp_mqtt_client_handle_t;

// I2C communications
/*
  I2C addresses, our settings:
    0x61 camera motor controller
    0x63 left wheel motor controller
    0x64 right wheel motor controller
    0x69 gy-912
    0x76 gy-912 BMP388
*/

#define I2C_MASTER_SCL_IO           GPIO_NUM_12  // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO           GPIO_NUM_15  // GPIO number for I2C master data
#define I2C_MASTER_NUM              I2C_NUM_0    // I2C port number for master dev
#define I2C_MASTER_FREQ1_HZ         400000       // I2C master clock frequency
#define I2C_MASTER_FREQ2_HZ         100000       // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS       1000	     // Timeout for I2C operations

#define CAM_MOTOR_I2C_ADDR          ((uint16_t) 0x61)
#define LEFT_MOTOR_I2C_ADDR         ((uint16_t) 0x63)
#define RIGHT_MOTOR_I2C_ADDR        ((uint16_t) 0x64)           
#define ICM20948_I2C_ADDR           ((uint16_t) 0x68) //0x69 AD0 High, 0x68 AD0 Low
#define BMP388_I2C_ADDR             ((uint16_t) 0x76) //0x77 AD0 High, 0x76 AD0 Low

// Wheel sensor GPIOs
// Assumptions (adjust if wiring differs):
//  - GPIO13 -> left wheel sensor passing over magnets
//  - GPIO14 -> right wheel sensor passing over magnets
// If reversed, swap the macros below in one place.
#define LEFT_WHEEL_SENSOR   GPIO_NUM_13
#define RIGHT_WHEEL_SENSOR  GPIO_NUM_14

// Circle board's RGB LED strip 
#define RGB_LED_RING_GPIO_PIN  GPIO_NUM_2
#define RGB_LED_RING_LED_COUNT 16 

typedef struct {
    uint8_t mac[6];
    uint8_t id;
} mac_id_map_t;

extern const mac_id_map_t g_mac_id_table[];
extern const size_t g_mac_id_table_size;
//extern const uint8_t g_debruijn_sequences[15][16];
extern const uint8_t g_debruijn_sequences[123][16];

// Per-sparknode motor configuration parameters
typedef struct {
    // Motor calibration parameters
    float left_motor_speed_multiplier;   // Adjust for motor differences (default: 1.0)
    float right_motor_speed_multiplier;  // Adjust for motor differences (default: 1.0)
    
    // Kick-start parameters for static friction
    uint8_t kick_speed;                  // Initial high speed to overcome static friction (default: 0x3F)
    uint16_t kick_duration_ms;           // How long to apply kick speed (default: 100ms)
    
    // Default motion parameters
    uint8_t default_drive_speed;         // Default speed for drive commands (default: 0x20)
    uint8_t default_turn_speed;          // Default speed for turn commands (default: 0x20)
    uint32_t default_drive_duration_ms;  // Default duration for drive commands (default: 2000ms)
    uint32_t default_turn_duration_ms;   // Default duration for turn commands (default: 1000ms)
    
    // Configuration status
    bool use_kick_start;                 // Whether to use kick-start mechanism
} sparknode_motor_config_t;

// Global variables
extern led_strip_handle_t g_rgb_led_ring_handle;
extern i2c_master_bus_handle_t g_i2c_bus_handle;
extern i2c_master_dev_handle_t g_cam_motor_dev_handle;
extern i2c_master_dev_handle_t g_left_motor_dev_handle;
extern i2c_master_dev_handle_t g_right_motor_dev_handle;
extern i2c_master_dev_handle_t g_bmp388_dev_handle;
extern i2c_master_dev_handle_t g_icm20948_dev_handle;
extern TaskHandle_t g_rgb_led_ring_task_handle;
extern esp_mqtt_client_handle_t g_mqtt_client;

// Global device identity variables
extern uint8_t g_sparknode_id;
extern char g_hostname[32];

// Global motor configuration instance
extern sparknode_motor_config_t g_motor_config;

// IMU sensor data structures
typedef enum {
    SENSOR_LOOP_STOP = 0,
    SENSOR_LOOP_INFINITE = 1,
    SENSOR_LOOP_COUNTED = 2
} sensor_loop_mode_t;

typedef enum {
    MODE_MAG_ACCEL_GYRO = 0,
    MODE_ACCEL_GYRO = 1,
    MODE_MAG = 2,
    MODE_MAG_ACCEL = 3
} sensor_stream_mode_t;

// IMU sensor loop control variables (defined in syncspark_config.c)
extern volatile sensor_loop_mode_t g_sensor_loop_mode;
extern volatile uint32_t g_sensor_loop_iterations;
extern volatile uint32_t g_sensor_loop_delay_ms;
extern volatile bool g_sensor_stream_enabled;
extern volatile uint32_t g_sensor_stream_period_ms;
extern volatile sensor_stream_mode_t g_sensor_stream_mode;

#endif
