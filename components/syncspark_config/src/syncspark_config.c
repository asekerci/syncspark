/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * syncspark_config.c - Configuration implementation for SynchroSpark nodes
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "syncspark_config.h"
#include "icm20948.h"  // For sensor data type definitions

led_strip_handle_t g_rgb_led_ring_handle = NULL;
i2c_master_bus_handle_t g_i2c_bus_handle = NULL;
i2c_master_dev_handle_t g_cam_motor_dev_handle = NULL;
i2c_master_dev_handle_t g_left_motor_dev_handle = NULL;
i2c_master_dev_handle_t g_right_motor_dev_handle = NULL;
i2c_master_dev_handle_t g_bmp388_dev_handle = NULL;
i2c_master_dev_handle_t g_icm20948_dev_handle = NULL;
TaskHandle_t g_rgb_led_ring_task_handle = NULL;
esp_mqtt_client_handle_t g_mqtt_client = NULL;

// Global device identity variables
uint8_t g_sparknode_id = 0;  // Will be set after reading the MAC address
char g_hostname[32];

// Inertial Measurement Unit (IMU) sensor global data structures and variables
icm20948_data_t g_sensor_data = {0};      // Current gyro + accel data
icm20948_mag_data_t g_mag_data = {0};     // Current magnetometer data
volatile sensor_loop_mode_t g_sensor_loop_mode = SENSOR_LOOP_STOP;
volatile uint32_t g_sensor_loop_iterations = 0;
volatile uint32_t g_sensor_loop_delay_ms = 50;
volatile bool g_sensor_stream_enabled = false;
volatile uint32_t g_sensor_stream_period_ms = 100;
volatile sensor_stream_mode_t g_sensor_stream_mode = MODE_MAG_ACCEL_GYRO;

// MAC To SparkNode ID mapping
// 0x00 is reserved for "not found" or "unknown" devices.
// 0xFF is reserved for broadcast messages.
const mac_id_map_t g_mac_id_table[] = {
    { {0xE4,0x65,0xB8,0x6F,0x46,0xE4}, 0x01 }, // Ahmet's SparkNode prototype
    { {0x3C,0x71,0xBF,0xE2,0xEF,0x30}, 0x02 }, // LED ring
    { {0xC0,0x49,0xEF,0xD2,0x0F,0xD0}, 0x03 }, // Ismet's ESP32-CAM board
    { {0x24,0x62,0xAB,0xD5,0xE4,0x94}, 0x04 }, // Ismet's ESP32-CAM board
    { {0xD4,0xE9,0xF4,0xA7,0x12,0xF0}, 0x05 }, // Ismet's ESP32-CAM board
    { {0xE4,0x65,0xB8,0x6F,0x44,0xD4}, 0x06 }, // Ahmet's SPL SparkNode
    { {0xE0,0x5A,0x1B,0xAB,0x47,0xC4}, 0x07 }, // Andy's SPL SparkNode
    { {0x80,0xF3,0xDA,0xAE,0x30,0xC0}, 0x08 }, // LED ring
    { {0x80,0xF3,0xDA,0xAD,0x55,0xA4}, 0x09 }, // Andy's ESP32-CAM board
    { {0x80,0xF3,0xDA,0xAD,0x64,0x90}, 0x0A }, // Ahmet's ESP32-CAM board
    { {0x6C,0xC8,0x40,0x34,0x5D,0xEC}, 0x0B }, // Optical Wireless Lab
    { {0x68,0x25,0xDD,0x2E,0x94,0x4C}, 0x0C }, // Optical Wireless Lab
    { {0x6C,0xC8,0x40,0x34,0x80,0x94}, 0x0D }, // Optical Wireless Lab
    { {0x6C,0xC8,0x40,0x34,0x03,0x04}, 0x0E }, // Optical Wireless Lab
    { {0x6C,0xC8,0x40,0x34,0x5F,0x38}, 0x0F }, // Optical Wireless Lab
    { {0x6C,0xC8,0x40,0x34,0x3E,0x14}, 0x10 }, // Optical Wireless Lab
    { {0x6C,0xC8,0x40,0x34,0x79,0x98}, 0x11 }, // LED ring 
    // Add more entries as needed
};

const size_t g_mac_id_table_size = sizeof(g_mac_id_table) / sizeof(g_mac_id_table[0]);

// Global motor configuration instance with default values (updated for better quality motors)
sparknode_motor_config_t g_motor_config = {
    .left_motor_speed_multiplier = 1.0f,
    .right_motor_speed_multiplier = 1.0f,
    .kick_speed = 0x2F,                    // Maximum kick speed (kept for manual override if needed)
    .kick_duration_ms = 50,                // kick duration in milliseconds
    .default_drive_speed = 0x20,           // Current working speed (32/63)
    .default_turn_speed = 0x20,            // Current working turn speed
    .default_drive_duration_ms = 2000,     // 2 second default drive
    .default_turn_duration_ms = 1000,      // 1 second default turn
    .use_kick_start = false                // Disabled by default for better quality motors
};

/*
// 16 LEDs, with three colors each, 
// 5 consecutive LEDs form a unique pattern
const uint8_t g_debruijn_sequences[15][16] = {
    { 2, 2, 1, 0, 0, 2, 0, 2, 1, 2, 2, 1, 2, 2, 0, 1 },
    { 0, 1, 0, 0, 0, 1, 0, 2, 1, 2, 0, 1, 2, 1, 0, 1 },
    { 2, 0, 1, 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 1, 0 },
    { 0, 2, 0, 1, 0, 2, 2, 0, 2, 2, 2, 0, 1, 0, 1, 2 },
    { 1, 1, 2, 1, 2, 0, 2, 2, 1, 0, 1, 2, 2, 2, 1, 1 },
    { 0, 1, 2, 0, 1, 1, 2, 2, 2, 0, 0, 1, 2, 1, 1, 0 },
    { 1, 0, 0, 2, 2, 0, 1, 1, 0, 2, 2, 2, 1, 2, 0, 0 },
    { 2, 1, 1, 0, 2, 0, 2, 2, 0, 0, 0, 2, 1, 2, 1, 0 },
    { 1, 2, 0, 0, 2, 2, 1, 1, 2, 0, 2, 1, 0, 0, 1, 1 },
    { 0, 1, 1, 0, 1, 2, 1, 2, 2, 2, 2, 0, 2, 1, 1, 1 },
    { 0, 1, 2, 0, 0, 0, 0, 0, 1, 2, 2, 0, 2, 0, 0, 2 },
    { 0, 0, 1, 0, 1, 1, 1, 0, 2, 1, 0, 1, 1, 2, 0, 1 },
    { 0, 1, 0, 2, 0, 0, 0, 1, 1, 2, 1, 1, 2, 2, 1, 1 },
    { 1, 1, 1, 2, 2, 0, 0, 2, 1, 0, 2, 2, 1, 2, 1, 2 },
    { 0, 2, 1, 1, 2, 1, 0, 0, 0, 0, 2, 0, 0, 1, 1, 0 }
};
*/

/* 
	Big swarm:
 	16 LEDs, with four colors each, 
	window size: 6 (6 consecutive LEDs form a unique pattern)
	python script: gen_debruijn_4c_128.py

	verify_debruijn_windows has identified 5 invalid sequences.
	I removed them. The remaining 123 sequences should be 
	sufficient for us.

	Suggested color assignment:
	(Gamma-balanced CMYW colors for robot LED IDs)
	const uint8_t COLOR_CYAN[3]    = {   0, 180, 255 }; 
	const uint8_t COLOR_MAGENTA[3] = { 255,   0, 200 }; 
	const uint8_t COLOR_YELLOW[3]  = { 255, 200,   0 };
	const uint8_t COLOR_WHITE[3]   = { 200, 200, 200 };
*/
const uint8_t g_debruijn_sequences[123][16] = {
    { 3, 3, 1, 1, 3, 3, 2, 1, 3, 3, 1, 0, 1, 0, 0, 0 }, // sparknode 0
    { 3, 2, 3, 0, 2, 2, 2, 0, 0, 3, 2, 2, 1, 2, 2, 1 }, // sparknode 1
    { 1, 3, 3, 3, 2, 2, 3, 0, 3, 3, 0, 1, 1, 1, 3, 2 }, // sparknode 2
    { 1, 3, 2, 2, 3, 3, 0, 3, 2, 1, 3, 0, 1, 0, 2, 2 }, // .....
    { 3, 0, 0, 2, 3, 2, 3, 0, 0, 2, 1, 0, 1, 3, 1, 3 },
    { 1, 1, 2, 3, 1, 1, 3, 0, 3, 1, 3, 1, 2, 3, 3, 0 },
    { 0, 1, 2, 3, 2, 1, 2, 0, 3, 1, 3, 3, 3, 0, 0, 2 },
    { 2, 2, 0, 2, 1, 0, 1, 1, 0, 2, 1, 3, 3, 1, 2, 1 },
    { 2, 3, 3, 2, 3, 1, 0, 0, 2, 0, 2, 0, 2, 1, 1, 1 },
    { 1, 1, 3, 3, 0, 3, 0, 3, 1, 1, 2, 3, 0, 0, 0, 3 },
    { 1, 1, 1, 0, 0, 2, 0, 1, 1, 2, 1, 1, 0, 2, 2, 0 },
    { 3, 3, 3, 1, 0, 1, 3, 3, 1, 0, 3, 1, 1, 2, 2, 1 },
    { 1, 3, 2, 2, 0, 0, 2, 1, 0, 3, 2, 3, 3, 1, 1, 1 },
    { 1, 0, 0, 0, 3, 0, 0, 2, 2, 3, 0, 2, 0, 3, 3, 3 },
    { 2, 2, 3, 0, 0, 1, 3, 0, 3, 0, 0, 3, 0, 0, 0, 2 },
    { 2, 2, 0, 3, 1, 3, 1, 3, 2, 1, 0, 3, 1, 0, 3, 1 },
    { 1, 3, 1, 0, 3, 1, 0, 1, 1, 1, 3, 1, 0, 0, 3, 2 },
    { 0, 3, 2, 1, 1, 2, 1, 1, 3, 2, 1, 0, 1, 1, 2, 2 },
    { 2, 3, 2, 0, 3, 0, 3, 0, 0, 2, 0, 0, 0, 0, 0, 2 },
    { 3, 2, 2, 2, 1, 2, 0, 2, 1, 3, 1, 1, 2, 3, 1, 0 },
    { 3, 2, 2, 1, 1, 0, 0, 3, 1, 2, 0, 3, 0, 1, 1, 0 },
    { 1, 1, 3, 3, 2, 3, 2, 3, 3, 2, 2, 2, 0, 1, 2, 1 },
    { 1, 0, 1, 0, 1, 3, 2, 3, 1, 1, 3, 1, 2, 0, 2, 3 },
    { 2, 3, 0, 2, 3, 1, 1, 0, 1, 0, 2, 3, 0, 2, 3, 3 },
    { 1, 2, 0, 0, 0, 0, 3, 3, 2, 2, 1, 3, 3, 2, 2, 1 },
    { 3, 1, 3, 3, 0, 2, 0, 1, 0, 0, 1, 1, 0, 0, 2, 2 },
    { 2, 1, 2, 3, 2, 2, 2, 2, 3, 3, 0, 2, 2, 1, 0, 3 },
    { 1, 2, 2, 2, 1, 2, 1, 0, 3, 2, 0, 3, 2, 0, 3, 3 },
    { 2, 2, 0, 1, 1, 0, 2, 0, 3, 0, 0, 3, 2, 3, 1, 1 },
    { 2, 2, 1, 2, 2, 0, 0, 0, 3, 2, 2, 0, 1, 1, 2, 2 },
    { 1, 3, 2, 0, 0, 0, 2, 3, 0, 1, 2, 2, 1, 0, 2, 3 },
    { 0, 3, 2, 3, 2, 2, 3, 2, 0, 2, 2, 2, 2, 1, 3, 3 },
    { 3, 3, 1, 1, 2, 0, 3, 0, 2, 2, 3, 2, 1, 0, 2, 2 },
    { 0, 3, 2, 2, 0, 2, 3, 2, 1, 1, 1, 2, 1, 0, 2, 3 },
    { 3, 2, 2, 0, 1, 3, 0, 3, 2, 2, 3, 1, 2, 0, 2, 2 },
    { 1, 2, 0, 2, 3, 2, 3, 1, 0, 1, 3, 2, 0, 3, 1, 1 },
    { 2, 0, 0, 3, 1, 2, 1, 1, 1, 0, 1, 2, 3, 1, 3, 1 },
    { 0, 1, 2, 1, 3, 0, 1, 2, 0, 1, 3, 3, 2, 1, 2, 3 },
    { 0, 0, 3, 1, 0, 3, 3, 3, 0, 2, 2, 0, 0, 1, 2, 1 },
    { 0, 0, 2, 0, 3, 3, 2, 0, 1, 2, 1, 0, 2, 1, 3, 2 },
    { 3, 2, 1, 2, 0, 2, 0, 1, 1, 3, 2, 2, 3, 2, 3, 0 },
    { 0, 1, 1, 2, 3, 2, 0, 0, 1, 3, 0, 2, 2, 0, 2, 0 },
    { 2, 1, 3, 3, 0, 0, 3, 3, 2, 0, 0, 2, 2, 0, 3, 0 },
    { 0, 3, 3, 3, 2, 3, 2, 2, 0, 3, 3, 0, 1, 3, 2, 3 },
    { 0, 3, 3, 0, 1, 0, 0, 2, 1, 3, 1, 0, 0, 2, 1, 1 },
    { 0, 1, 2, 3, 3, 1, 3, 3, 2, 1, 1, 0, 3, 0, 3, 2 },
    { 2, 2, 2, 0, 3, 0, 1, 2, 2, 0, 3, 3, 3, 2, 1, 2 },
    { 2, 2, 1, 3, 0, 2, 1, 0, 1, 2, 0, 3, 3, 3, 3, 1 },
    { 1, 2, 1, 2, 1, 0, 2, 0, 2, 2, 3, 3, 3, 1, 2, 3 },
    { 3, 1, 1, 2, 0, 1, 0, 1, 3, 1, 2, 0, 3, 1, 0, 2 },
    { 3, 0, 1, 1, 0, 2, 2, 2, 3, 1, 3, 0, 1, 1, 3, 3 },
    { 2, 2, 3, 2, 3, 1, 3, 1, 3, 1, 1, 3, 3, 1, 3, 1 },
    { 1, 2, 1, 0, 0, 0, 1, 3, 0, 1, 3, 3, 3, 2, 3, 3 },
    { 0, 1, 3, 0, 3, 3, 2, 1, 0, 2, 1, 1, 3, 1, 3, 1 },
    { 3, 0, 1, 3, 1, 1, 2, 2, 3, 3, 2, 1, 0, 0, 1, 0 },
    { 1, 2, 1, 0, 1, 0, 1, 1, 0, 3, 0, 1, 0, 0, 0, 1 },
    { 0, 1, 0, 1, 0, 2, 0, 0, 3, 3, 2, 1, 1, 1, 1, 1 },
    { 3, 2, 3, 3, 0, 0, 0, 1, 2, 2, 2, 2, 2, 2, 3, 0 },
    { 3, 2, 0, 3, 3, 0, 3, 3, 3, 3, 3, 0, 2, 0, 0, 2 },
    { 3, 3, 3, 2, 0, 0, 1, 2, 0, 1, 2, 2, 2, 3, 3, 2 },
    { 2, 0, 1, 3, 1, 0, 3, 0, 0, 0, 0, 0, 1, 0, 2, 2 },
    { 1, 1, 0, 3, 2, 0, 2, 2, 3, 1, 1, 3, 2, 3, 3, 3 },
    { 1, 3, 2, 0, 1, 3, 3, 3, 3, 2, 2, 2, 3, 2, 3, 3 },
    { 3, 1, 2, 0, 1, 0, 0, 2, 3, 3, 0, 2, 1, 1, 3, 3 },
    { 3, 0, 2, 2, 0, 1, 0, 1, 2, 1, 0, 3, 3, 1, 3, 0 },
    { 3, 0, 2, 3, 2, 2, 2, 1, 0, 1, 2, 1, 3, 2, 1, 1 },
    { 0, 1, 0, 1, 3, 0, 2, 1, 2, 1, 2, 2, 1, 2, 3, 0 },
    { 3, 2, 2, 0, 0, 0, 1, 0, 0, 1, 2, 0, 3, 2, 2, 3 },
    { 1, 1, 2, 3, 2, 2, 0, 0, 3, 0, 1, 1, 2, 1, 3, 3 },
    { 3, 3, 2, 2, 0, 3, 0, 0, 2, 1, 1, 3, 2, 3, 1, 0 },
    { 0, 0, 0, 3, 0, 1, 0, 1, 0, 3, 2, 3, 0, 2, 1, 3 },
    { 2, 3, 1, 3, 0, 0, 2, 2, 1, 1, 0, 3, 1, 1, 1, 1 },
    { 1, 3, 1, 1, 1, 0, 2, 0, 2, 1, 0, 2, 2, 1, 2, 1 },
    { 2, 3, 1, 1, 0, 2, 1, 2, 0, 0, 0, 1, 2, 1, 2, 3 },
    { 1, 0, 0, 0, 0, 3, 0, 2, 1, 2, 0, 2, 3, 3, 2, 2 },
    { 2, 0, 3, 0, 2, 0, 2, 3, 0, 1, 1, 3, 1, 2, 2, 0 },
    { 3, 1, 0, 0, 1, 3, 2, 2, 0, 3, 2, 2, 1, 3, 0, 1 },
    { 0, 1, 2, 1, 2, 1, 1, 2, 1, 3, 2, 2, 0, 2, 1, 2 },
    { 2, 2, 1, 3, 1, 2, 1, 3, 1, 3, 0, 1, 2, 3, 2, 3 },
    { 3, 0, 1, 3, 0, 3, 1, 1, 1, 3, 2, 3, 0, 0, 3, 0 },
    { 1, 2, 3, 2, 2, 1, 2, 1, 2, 1, 3, 0, 2, 0, 1, 3 },
    { 0, 0, 0, 0, 1, 3, 2, 1, 1, 0, 0, 0, 3, 1, 3, 1 },
    { 0, 0, 0, 2, 1, 1, 1, 0, 2, 3, 2, 3, 2, 0, 2, 3 },
    { 2, 2, 2, 3, 2, 2, 3, 3, 3, 0, 1, 0, 2, 3, 1, 2 },
    { 0, 0, 3, 3, 0, 0, 1, 1, 1, 0, 1, 3, 0, 0, 1, 2 },
    { 1, 2, 0, 2, 1, 1, 2, 2, 2, 0, 3, 2, 3, 1, 2, 1 },
    { 0, 1, 3, 3, 1, 3, 0, 1, 3, 0, 1, 0, 3, 3, 3, 3 },
    { 1, 3, 3, 2, 3, 1, 1, 1, 3, 1, 1, 2, 1, 3, 0, 0 },
    { 3, 0, 3, 1, 3, 2, 1, 3, 1, 2, 0, 1, 2, 0, 0, 2 },
    { 1, 0, 3, 2, 1, 0, 0, 0, 2, 1, 2, 2, 2, 3, 1, 1 },
    { 2, 0, 1, 3, 3, 1, 1, 3, 2, 0, 1, 1, 3, 3, 2, 0 },
    { 0, 2, 3, 1, 3, 3, 2, 0, 3, 1, 2, 1, 0, 1, 3, 3 },
    { 3, 1, 2, 2, 3, 1, 0, 0, 3, 1, 3, 0, 2, 2, 3, 0 },
    { 2, 2, 3, 1, 0, 2, 1, 1, 1, 3, 2, 0, 2, 1, 3, 2 },
    { 3, 0, 1, 2, 2, 3, 3, 1, 0, 1, 1, 2, 0, 0, 1, 3 },
    { 2, 0, 3, 2, 3, 3, 3, 0, 3, 2, 0, 3, 0, 0, 1, 1 },
    { 3, 2, 0, 0, 3, 2, 0, 1, 1, 0, 0, 3, 0, 3, 3, 2 },
    { 2, 3, 3, 3, 2, 1, 3, 0, 0, 2, 3, 0, 0, 1, 1, 0 },
    { 0, 2, 1, 0, 0, 2, 3, 1, 0, 0, 1, 2, 1, 3, 3, 3 },
    { 0, 2, 3, 2, 0, 1, 0, 3, 1, 2, 0, 1, 3, 2, 0, 1 },
    { 3, 2, 1, 1, 3, 2, 0, 0, 3, 3, 3, 1, 1, 1, 3, 3 },
    { 1, 2, 3, 0, 1, 3, 0, 2, 0, 3, 1, 0, 3, 0, 1, 1 },
    { 1, 1, 1, 1, 0, 0, 3, 3, 1, 3, 2, 1, 2, 1, 2, 0 },
    { 3, 3, 0, 1, 0, 3, 0, 2, 3, 3, 0, 0, 3, 2, 1, 1 },
    { 2, 0, 2, 3, 0, 3, 0, 1, 2, 1, 1, 0, 1, 1, 1, 2 },
    { 1, 0, 2, 3, 0, 0, 2, 0, 2, 3, 3, 1, 3, 0, 2, 3 },
    { 2, 3, 3, 0, 1, 3, 1, 3, 1, 3, 0, 0, 3, 1, 3, 3 },
    { 3, 2, 2, 1, 2, 3, 1, 2, 3, 1, 1, 2, 1, 2, 3, 3 },
    { 0, 0, 1, 1, 3, 1, 0, 1, 2, 0, 2, 0, 3, 2, 0, 0 },
    { 2, 2, 1, 1, 3, 3, 1, 0, 0, 1, 0, 0, 0, 0, 2, 0 },
    { 1, 2, 0, 2, 0, 0, 2, 1, 2, 0, 3, 3, 0, 2, 3, 3 },
    { 1, 1, 2, 3, 3, 1, 0, 2, 0, 1, 2, 2, 3, 0, 2, 2 },
    { 2, 2, 0, 1, 0, 3, 3, 0, 2, 0, 2, 1, 2, 3, 1, 3 },
    { 1, 1, 0, 3, 3, 2, 3, 3, 2, 0, 3, 3, 2, 3, 0, 1 },
    { 0, 1, 0, 0, 3, 3, 0, 3, 2, 2, 2, 3, 3, 1, 2, 3 },
    { 2, 2, 0, 1, 3, 2, 2, 3, 1, 3, 2, 3, 0, 1, 0, 1 },
    { 1, 0, 1, 2, 2, 2, 1, 3, 2, 1, 0, 2, 3, 2, 2, 3 },
    { 0, 0, 0, 3, 1, 0, 1, 0, 2, 1, 0, 3, 0, 3, 3, 0 },
    { 0, 2, 0, 3, 1, 3, 2, 0, 2, 2, 0, 3, 1, 1, 3, 0 },
    { 2, 0, 2, 2, 3, 0, 1, 2, 3, 1, 0, 2, 0, 3, 2, 2 },
    { 0, 0, 3, 2, 0, 2, 1, 0, 3, 1, 3, 2, 2, 1, 1, 3 },
    { 2, 0, 0, 1, 3, 2, 3, 2, 3, 1, 2, 3, 0, 2, 3, 0 },
    { 0, 3, 1, 0, 1, 3, 1, 1, 1, 1, 1, 1, 2, 0, 0, 3 },
};

