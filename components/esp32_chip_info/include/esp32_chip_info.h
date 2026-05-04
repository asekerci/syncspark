/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32_chip_info.h - ESP32 chip information utilities
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

#ifndef GET_CHIP_INFO_H
#define GET_CHIP_INFO_H

#include "esp_system.h"
#include "esp_clk_tree.h"
#include "esp_psram.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "spi_flash_mmap.h"    // Replacement for esp_spi_flash.h
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_chip_info.h"

// Function Prototypes:
void get_chip_info(void);


#endif