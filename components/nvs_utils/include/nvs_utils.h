/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * nvs_utils.h - Non-volatile storage utilities for SynchroSpark
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

#ifndef NVS_UTILS_H
#define NVS_UTILS_H

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"


// Function prototypes
esp_err_t initialize_nvs(void);
esp_err_t store_data_in_nvs(const char *key, const char *data);
esp_err_t retrieve_data_from_nvs(const char *key, char *data, size_t data_size);

#endif // NVS_UTILS_H
