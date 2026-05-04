/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_ledring.c - LED ring control node for SynchroSpark
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
 */

#define OTA_UPDATE_ENABLED
This program assumes that sparknode_ota_updater is already placed in the factory partition of 
the device.  
*/

#include <stdio.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "esp32_chip_info.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "rgb_led_ring.h"  // Now includes set_leds_from_debruijn_sequence()
#include "ota_utils.h"
#include "wifi_net.h"
#include "wifi_credentials.h"
#include "network_config.h"

/*
Attention:

In our standard design, the RGB LED ring connects to GPIO 2 of the ESP32-CAM board. However, 
GPIO 2 is one of the ESP32-CAM's strapping pins, meaning that if it is pulled high during boot,
the ESP32-CAM will not enter flashing mode. To address this issue, Ismet has designed a small 
circuit on the camera back board that ensures GPIO 2 remains low during startup.

After flashing "sparknode_ota_updater" to the device via USB cable connection, we can safely 
continue using GPIO 2 to control the RGB LED ring since subsequent firmware updates use the OTA
method. If USB flashing is needed again, either the back board circuit must be present to
hold GPIO 2 low during boot, or the RGB LED ring must be unplugged from GPIO 2 before flashing
the device.
*/

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) { // Blink the red LED continuously
    blink_esp32cam_red_led (200, 1000);
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
    vTaskDelay(pdMS_TO_TICKS(10000)); 
  }
} // task_udp_heartbeat()

void app_main (void) 
{
  char *TAG = "sparknode_ledring";
  int rgb_led_ring_gpio_pin = RGB_LED_RING_GPIO_PIN;

  // We call set_next_boot_to_factory() at the very beginning of app_main()
  // to ensure that the next boot will use the factory partition.
  #ifdef OTA_UPDATE_ENABLED
    ESP_LOGI(TAG, "OTA update enabled");
    set_next_boot_to_factory();
  #endif // OTA_UPDATE_ENABLED
  
  initialize_esp32cam_leds(); test_esp32cam_leds();       
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
  
  // Prepare g_hostname based on SparkNode ID
   if (g_sparknode_id != 0x00) {
    snprintf(g_hostname, sizeof(g_hostname), "sparknode%02d", g_sparknode_id);
  } else {
    // Fallback to MAC address if SparkNode ID is not available
    snprintf(g_hostname, sizeof(g_hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }
  
  // Initialize WiFi with custom g_hostname
  initialize_wifi_sta_with_hostname(WIFI_SSID, WIFI_PASS, g_hostname);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();
  indicate_success_esp32cam_flash_led();
  //int64_t *start_time = malloc(sizeof(int64_t)); 
  //*start_time = esp_timer_get_time();  // For measuring elapsed time
  get_chip_info(); fflush(stdout);
  get_reset_reason(); 

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }

  // Light-up the RGB LED ring    
  ESP_LOGI(TAG, "RGB LED ring control pin is %d", rgb_led_ring_gpio_pin);
  initialize_rgb_led_ring(&g_rgb_led_ring_handle, rgb_led_ring_gpio_pin,RGB_LED_RING_LED_COUNT);
  set_leds_from_debruijn_sequence(g_rgb_led_ring_handle, g_sparknode_id);
    
  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }

  if (pdPASS != xTaskCreate(&task_udp_heartbeat, "task_udp_heartbeat", 2048, NULL, 5, NULL)) {
    ESP_LOGE(TAG, "Failed to create UDP heartbeat task");
  } 
}	// app_main()
