/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_sample_ota_app.c - Sample OTA application for SynchroSpark
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
 *
 * Important:
 * Do not flash this program to the device, it will be downloaded wirelessly by the OTA updater.
 * This program assumes that sparknode_ota_updater is already placed in the factory partition of
 * the device. See the main README.md for more information.
 */

#define OTA_UPDATE_ENABLED

#include <stdio.h>
#include <string.h>
#include "esp32_chip_info.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "ota_utils.h"
#include "wifi_net.h"
#include "esp_netif.h"

// Create your own file to define these macros:
// #define WIFI_SSID "your wifi ssid here"
// #define WIFI_PASS "your wifi password here"
#include "wifi_credentials.h"

// Contains definitions of DESTINATION_IP_ADDR, DESTINATION_UDP_PORT, 
// LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT, UDP_PACKET_SIZE,
// CHECKSUM_FILE_URL and BINARY_FILE_URL
#include "network_config.h" 

static uint8_t g_sparknode_id = 0; // Will be set after reading the MAC address

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) {
    blink_esp32cam_red_led (200, 1200);	
  }
} // task_blink_esp32cam_red_led()

void task_udp_heartbeat(void *pvParameter) 
{
  static uint32_t i = 1;
  while (1) {
    blink_esp32cam_flash_led(9, 100);
    blink_esp32cam_flash_led(9, 100);
    
    // Get WiFi RSSI
    int rssi = -999;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      rssi = ap_info.rssi;
    }
    ESP_LOGI("UDP heartbeat", "(id %d) #%" PRIu32 " RSSI=%d dBm", 
             									g_sparknode_id, i++, rssi);
    vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
  }
} // task_udp_heartbeat()

void app_main(void) 
{
  static const char *TAG = "sparknode_sample_ota_app";

  // We call set_next_boot_to_factory() at the very beginning of app_main()
  // to ensure that the next boot will use the factory partition.
  #ifdef OTA_UPDATE_ENABLED
    ESP_LOGI(TAG, "OTA update enabled");
    set_next_boot_to_factory();
  #endif // OTA_UPDATE_ENABLED
    
  initialize_esp32cam_leds(); // Initialize GPIO pins for built-in LEDs
  test_esp32cam_leds();       // Blink the Flash and red LEDs
  if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        indicate_error_esp32cam_red_led();  
        return;
  }
  
  // Get MAC address and determine SparkNode ID before WiFi initialization
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  g_sparknode_id = mac_to_id(mac);
  if (g_sparknode_id == 0x00) {
      ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
  } else {
      ESP_LOGI(TAG, "SparkNode ID is %d", g_sparknode_id);
  }
  
  // Prepare hostname based on SparkNode ID
  char hostname[32];
  if (g_sparknode_id != 0x00) {
    snprintf(hostname, sizeof(hostname), "sparknode%02d", g_sparknode_id);
  } else {
    // Fallback to MAC address if SparkNode ID is not available
    snprintf(hostname, sizeof(hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }
  
  // Initialize WiFi with custom hostname
  initialize_wifi_sta_with_hostname(WIFI_SSID, WIFI_PASS, hostname);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();
  indicate_success_esp32cam_flash_led();
  get_chip_info(); fflush(stdout);
  get_reset_reason();

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }
  
  if (pdPASS != xTaskCreate(&task_udp_heartbeat, "task_udp_heartbeat", 2048, NULL, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create UDP heartbeat task");
  } 

  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
      ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }
} // app_main()
