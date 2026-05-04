/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sys_utils.c - System utilities implementation for SynchroSpark
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

#include <stddef.h>
#include "esp_log.h"
#include "esp_system.h"
#include "syncspark_config.h"
#include "sys_utils.h"

uint8_t mac_to_id(const uint8_t mac[6]) {
    for (size_t i = 0; i < g_mac_id_table_size; ++i) {
        int match = 1;
        for (int j = 0; j < 6; ++j) {
            if (g_mac_id_table[i].mac[j] != mac[j]) {
                match = 0;
                break;
            }
        }
        if (match) return g_mac_id_table[i].id;
    }
    return 0x00; // Not found
}

uint8_t get_debruijn_sequence_index(uint8_t sparknode_id) {
    if (sparknode_id == 0x00 || sparknode_id == 0xFF) {
        return 0; // Default to first sequence for invalid IDs
    }

    // Map sparknode ID to sequence index (1-128 maps to 0-127)
    // Cycle through available sequences if we have more than 128 nodes
    return (sparknode_id - 1) % 128;
}

void get_reset_reason(void)
{
    char *TAG = "Reset_Reason";
    
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_BROWNOUT:
            ESP_LOGW(TAG, "Brownout detected on last reset");
            break;
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "Power-on reset");
            break;
        case ESP_RST_SW:
            ESP_LOGI(TAG, "Software reset");
            break;
        case ESP_RST_PANIC:
            ESP_LOGE(TAG, "Panic reset (probably crash)");
            break;
        case ESP_RST_EXT:
            ESP_LOGI(TAG, "External reset");
            break;
        case ESP_RST_INT_WDT:
            ESP_LOGI(TAG, "Interrupt watchdog reset");
            break;
        case ESP_RST_TASK_WDT:
            ESP_LOGI(TAG, "Task watchdog reset");
            break;
        case ESP_RST_DEEPSLEEP:
            ESP_LOGI(TAG, "Deep sleep wakeup");
            break;
        case ESP_RST_UNKNOWN:
            ESP_LOGI(TAG, "Unknown reason");
            break;
        default:
            ESP_LOGI(TAG, "Reset reason: %d", reason);
            break;
    }
} // get_reset_reason()
