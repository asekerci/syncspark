/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * nvs_utils.c - Non-volatile storage utilities implementation for SynchroSpark
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

#include "nvs_utils.h"

static const char *TAG = "nvs_utils";

esp_err_t initialize_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || 
    	                 err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
} // initialize_nvs()

esp_err_t store_data_in_nvs(const char *key, const char *data) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(my_handle, key, data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) setting NVS data!", esp_err_to_name(err));
    }
    nvs_close(my_handle);
    return err;
} // store_data_in_nvs()

esp_err_t retrieve_data_from_nvs(const char *key, char *data, size_t data_size) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }
    size_t required_size = data_size;
    err = nvs_get_str(my_handle, key, data, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) getting NVS data!", esp_err_to_name(err));
    }
    nvs_close(my_handle);
    return err;
} // retrieve_data_from_nvs()
