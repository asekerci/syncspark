/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32cam_leds.c - ESP32-CAM LED control implementation for SynchroSpark
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

// esp32cam_leds.c is responsible for controlling the esp32cam's two built-in LEDs.
// See esp32cam_leds.h for the function prototypes and a summary of the functions.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "esp32cam_leds.h"

static const char *TAG = "led_control";

void initialize_esp32cam_leds(void) {
    ESP_LOGI(TAG, "Initializing GPIOs for LEDs");
    gpio_reset_pin(FLASH_LED_GPIO);
    gpio_set_direction(FLASH_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FLASH_LED_GPIO, 0);      // Ensure flash LED is off

    gpio_reset_pin(RED_LED_GPIO);
    gpio_set_direction(RED_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RED_LED_GPIO, 1);        // Ensure red LED is off (active-low)
} // initialize_esp32cam_leds()

void test_esp32cam_leds(void) {
    int i;
    ESP_LOGI(TAG, "Testing LEDS");
    for (i=0; i<3; i++) {
        blink_esp32cam_flash_led(9, 200);
    }
    for (i=0; i<3; i++) {
        blink_esp32cam_red_led(20, 200);
    }
} // test_esp32cam_leds()

void blink_esp32cam_flash_led(int led_on_time_ms, int led_off_time_ms) {
    gpio_set_level(FLASH_LED_GPIO, 1);  // Turn on the flash LED
    vTaskDelay(pdMS_TO_TICKS(led_on_time_ms));    
    gpio_set_level(FLASH_LED_GPIO, 0);  // Turn off the flash LED
    vTaskDelay(pdMS_TO_TICKS(led_off_time_ms)); 
} // blink_esp32cam_flash_led()

void indicate_success_esp32cam_flash_led(void) {
    ESP_LOGI(TAG, "Indicating success with flash LED");
    for (int i=0; i<3; i++) {
        blink_esp32cam_flash_led(9, 100);
    }
} // indicate_success_esp32cam_flash_led()

void indicate_error_esp32cam_red_led(void) {
    ESP_LOGI(TAG, "Indicating error with red LED");
    blink_esp32cam_red_led(2000, 0);       
} // indicate_error_esp32cam_red_led()

void indicate_failure_esp32cam_red_led(void) {
    ESP_LOGI(TAG, "Indicating failure with red LED");
    while (1) { 
        blink_esp32cam_red_led(100, 100);     
    }
} // indicate_failure_esp32cam_red_led()

void blink_esp32cam_red_led(int led_on_time_ms, int led_off_time_ms) {
    gpio_set_level(RED_LED_GPIO, 0);    // Turn on the red LED (active-low)
    vTaskDelay(pdMS_TO_TICKS(led_on_time_ms));
    gpio_set_level(RED_LED_GPIO, 1);    // Turn off the red LED (active-low)
    vTaskDelay(pdMS_TO_TICKS(led_off_time_ms)); 
} // blink_esp32cam_red_led()

