/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * ota_utils.h - Over-the-air update utilities for SynchroSpark
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

#ifndef OTA_UTILS_H
#define OTA_UTILS_H

#include <stdio.h>
#include <stddef.h> // Include this header for size_t
#include <stdlib.h>
#include <string.h>
#include <mbedtls/md5.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_utils.h"
#include "esp32cam_leds.h"

typedef enum {
    FILE_TYPE_TEXT,
    FILE_TYPE_BINARY
} file_type_t;

typedef enum {
    WRITE_TO_NVS,
    WRITE_TO_OTA_0,
    WRITE_TO_OTA_1,
    WRITE_TO_VARIABLE
} write_destination_t;

void calculate_md5_checksum(const char *partition_label, 
                                    char *checksum, size_t total_bytes_written);
esp_err_t read_remote_file_and_write(const char *url, file_type_t file_type, 
                                    write_destination_t write_destination, 
                                    size_t *total_bytes_written, char *variable);
esp_err_t check_for_update_and_restart(const char *url);
esp_err_t read_remote_checksum(const char *url, char *checksum);
void log_elapsed_time(int64_t start_time);
void set_next_boot_to_factory(void);

#endif
