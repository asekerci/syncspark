/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32cam_leds.h - ESP32-CAM LED control header
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

#ifndef ESP32CAM_LEDS_H
#define ESP32CAM_LEDS_H

#include "driver/gpio.h"

#define FLASH_LED_GPIO 	GPIO_NUM_4   // The flash LED on ESP32-CAM
#define RED_LED_GPIO 	GPIO_NUM_33  // The red LED on ESP32-CAM

// Function prototypes
void initialize_esp32cam_leds(void);            // initializes the GPIO pins for the LEDs.
void test_esp32cam_leds(void);                  // tests the LEDs.
void blink_esp32cam_flash_led(int led_on_time_ms, int led_off_time_ms);
void blink_esp32cam_red_led(int led_on_time_ms, int led_off_time_ms); 
void indicate_success_esp32cam_flash_led(void);
void indicate_error_esp32cam_red_led(void);     
void indicate_failure_esp32cam_red_led(void);   


#endif // ESP32CAM_LEDS_H
