/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * rgb_led_ring.h - RGB LED ring control for SynchroSpark robots
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

#ifndef RGB_LED_RING_H
#define RGB_LED_RING_H

#include "esp_log.h"
#include "led_strip.h"

// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RGB_LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

// Function prototypes - global functions
void initialize_rgb_led_ring(led_strip_handle_t *rgb_led_ring_handle, 
                                    int rgb_led_ring_gpio_pin, int rgb_led_ring_led_count);
void set_leds_from_debruijn_sequence(led_strip_handle_t led_handle, uint8_t sparknode_id);
void set_leds_from_debruijn_sequence_progressive(led_strip_handle_t led_handle, 
                                                   uint8_t sparknode_id, uint32_t delay_ms);

#endif // RGB_LED_RING_H
