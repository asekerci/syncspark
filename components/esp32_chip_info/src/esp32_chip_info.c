/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32_chip_info.c - ESP32 chip information utilities implementation
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

#include "esp32_chip_info.h"
 
static const char *TAG = "esp32_info";

void get_chip_info(void) 
{
    uint32_t cpu_freq_hz;
    uint8_t mac[6];
    
    esp_err_t err = esp_clk_tree_src_get_freq_hz(SOC_CPU_CLK_SRC_PLL, SOC_MOD_CLK_RTC_FAST, &cpu_freq_hz);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CPU Clock Frequency: %ld MHz", cpu_freq_hz / 1000000);
    } else {
        ESP_LOGE(TAG, "Failed to get CPU clock frequency: %s", esp_err_to_name(err));
    }

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "This is an %s chip with %d CPU core(s) with %s%s%s%s ",
           CONFIG_IDF_TARGET, chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? " WiFi" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? " BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? " BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? " 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    ESP_LOGI(TAG, "Silicon revision: v%d.%d ", major_rev, minor_rev);
    
    // Obtain the base MAC address
    esp_efuse_mac_get_default(mac);
    ESP_LOGI(TAG, "Base MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // Print SRAM Size
    /* 
        The following constants are used in ESP32 to specify memory allocation capabilities. 
        They define the types of memory that can be allocated for different purposes, and 
        they help control how memory is used in the ESP32 system:

        MALLOC_CAP_8BIT:     This constant refers to memory that supports 8-bit byte-aligned access. 
                             It allows you to allocate memory that can be accessed in 8-bit chunks 
                             (i.e., byte-by-byte). Most general-purpose memory can be allocated with 
                             this flag.
        MALLOC_CAP_32BIT:    This refers to memory that supports 32-bit word-aligned access. 
                             It allows you to allocate memory that is aligned on 32-bit boundaries, 
                             which can be useful for performance in some scenarios where 32-bit 
                             accesses are needed (e.g., working with certain peripherals or 
                             optimized code that handles 32-bit values).

        MALLOC_CAP_INTERNAL: This refers to memory that is located internally on the ESP32 chip, as 
                             opposed to external memory (like PSRAM). It usually refers to the 
                             fastest memory available, such as the DRAM that is built into the ESP32. 
                             This type of memory is often used for time-sensitive operations or 
                             where low-latency access is required.
    */
    size_t total_internal = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "SRAM Size: %d KB (Total), %d KB (Free)", total_internal / 1024, free_internal / 1024);
    ESP_LOGI(TAG, "Minimum free heap size: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Free heap: %ld", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Free internal heap: %ld", esp_get_free_internal_heap_size());

    // Check if PSRAM is available and print PSRAM Size
    if (esp_psram_is_initialized()) {
        size_t psram_size = esp_psram_get_size();
        ESP_LOGI(TAG, "Total PSRAM: %d MB", psram_size / (1024 * 1024));
        size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        ESP_LOGI(TAG, "Free PSRAM: %d MB", free_psram / (1024 * 1024));
    } else {
        ESP_LOGE(TAG, "PSRAM  is not available");
    }

    // Get the flash memory size (in bytes)
    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash memory: %" PRIu32 " MB %s ", flash_size / (uint32_t)(1024 * 1024),
                                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    } else {
        ESP_LOGE(TAG, "Get flash size failed");
    }

    // Information about network interfaces
    /*
        MAC address types:
        ESP_MAC_WIFI_STA: Station MAC address (default for Wi-Fi clients).
        ESP_MAC_WIFI_SOFTAP: SoftAP MAC address.
        ESP_MAC_BT: Bluetooth MAC address (if Bluetooth is enabled).
        ESP_MAC_ETH: Ethernet MAC address (if Ethernet is used).
    */
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "WiFi MAC Address= %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_read_mac(mac, ESP_MAC_BT);
    ESP_LOGI(TAG, "BlueTooth MAC Address= %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
} // get_chip_info()

