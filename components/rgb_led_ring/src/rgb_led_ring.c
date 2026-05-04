/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * rgb_led_ring.c - RGB LED ring control implementation for SynchroSpark robots
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "syncspark_config.h"
#include "sys_utils.h"  // For get_debruijn_sequence_index() and debruijn_sequences[]
#include "rgb_led_ring.h"

static char *TAG = "RGB LED ring";

void initialize_rgb_led_ring(led_strip_handle_t *rgb_led_ring_handle, int rgb_led_ring_gpio_pin,
                                            int rgb_led_ring_led_count)
{
    led_strip_config_t ring_config = {
        .strip_gpio_num = rgb_led_ring_gpio_pin,                
        .max_leds = rgb_led_ring_led_count,                          // The number of LEDs in the strip
        .led_model = LED_MODEL_WS2812,        
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // The color order of the strip: GRB
        .flags = {
            .invert_out = false,                                      // Don't invert the output signal
        }
    };

    // RGB LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,            // Different clock source can lead to
                                                   // different power consumption.
        .resolution_hz = RGB_LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,                   // The memory size of each RMT channel, in
                                                   // words (4 bytes).
        .flags = {
            .with_dma = false,                     // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&ring_config, &rmt_config, rgb_led_ring_handle));
    ESP_LOGI(TAG, "Created the RGB LED strip object with RMT backend");
} // initialize_rgb_led_strip()

// Function: Light up LEDs based on sparknode ID's De Bruijn sequence
void set_leds_from_debruijn_sequence(led_strip_handle_t led_handle, uint8_t sparknode_id) 
{
    if (led_handle == NULL) {
        return;
    }
    
    uint8_t sequence_index = get_debruijn_sequence_index(sparknode_id);
    ESP_LOGI(TAG, "Using De Bruijn sequence index %d", sequence_index);
    
    const uint8_t *sequence = g_debruijn_sequences[sequence_index];

    // Gamma-balanced CMYW colors for robot LED IDs
    // RGB values for each color
    const uint8_t colors[4][3] = {
        {  0, 18, 25 }, // CYAN    (0)
        { 25,  0, 30 }, // MAGENTA (1)
        { 23, 18,  0 }, // YELLOW  (2)
        {  4, 20,  4 }  // SOFT-GREEN (3)
    };

    // Set each LED according to the De Bruijn sequence
    for (int i = 0; i < RGB_LED_RING_LED_COUNT; i++) {
        uint8_t color_index = sequence[i];
        if (color_index < 4) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, i, 
                colors[color_index][0], 
                colors[color_index][1], 
                colors[color_index][2]
            ));
        }
    }
    
    // Refresh the LED strip to show the changes
    ESP_ERROR_CHECK(led_strip_refresh(led_handle));
} // set_leds_from_debruijn_sequence()

// Function: Light up LEDs progressively based on sparknode ID's De Bruijn sequence
void set_leds_from_debruijn_sequence_progressive(led_strip_handle_t led_handle, uint8_t sparknode_id, uint32_t delay_ms) 
{
    if (led_handle == NULL) {
        return;
    }
    
    uint8_t sequence_index = get_debruijn_sequence_index(sparknode_id);
    ESP_LOGI(TAG, "Using De Bruijn sequence index %d for progressive lighting", sequence_index);
    
    const uint8_t *sequence = g_debruijn_sequences[sequence_index];

    // Gamma-balanced CMYW colors for robot LED IDs
    // RGB values for each color
    const uint8_t colors[4][3] = {
        {  0, 18, 25 }, // CYAN    (0)
        { 25,  0, 30 }, // MAGENTA (1)
        { 23, 18,  0 }, // YELLOW  (2)
        {  4, 20,  4 }  // SOFT-GREEN (3)
    };

    // Clear all LEDs first
    ESP_ERROR_CHECK(led_strip_clear(led_handle));
    ESP_ERROR_CHECK(led_strip_refresh(led_handle));

    // Set each LED progressively with delay
    for (int i = 0; i < RGB_LED_RING_LED_COUNT; i++) {
        uint8_t color_index = sequence[i];
        if (color_index < 4) {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_handle, i, 
                colors[color_index][0], 
                colors[color_index][1], 
                colors[color_index][2]
            ));
            
            // Refresh to show this LED
            ESP_ERROR_CHECK(led_strip_refresh(led_handle));
            
            // Wait before lighting the next LED (except for the last one)
            if (i < RGB_LED_RING_LED_COUNT - 1) {
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
            }
        }
    }
} // set_leds_from_debruijn_sequence_progressive()

