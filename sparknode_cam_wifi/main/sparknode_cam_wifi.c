/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_cam_wifi.c - WiFi camera streaming node for SynchroSpark
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
 * the device.
 */

#define OTA_UPDATE_ENABLED

#include <stdio.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "ota_utils.h"
#include "esp32_chip_info.h"
#include "esp32cam_leds.h"
#include "esp32cam_ov2640.h"

// Create your own file to define these macros:
// #define WIFI_SSID "your wifi ssid here"
// #define WIFI_PASS "your wifi password here"
#include "wifi_credentials.h"

// Contains definitions for DESTINATION_IP_ADDR, DESTINATION_UDP_PORT, 
// LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT and UDP_PACKET_SIZE
#include "network_config.h" 

#include "wifi_net.h"

static const char *TAG = "sparknode_cam_wifi";
uint8_t g_sparknode_id = 0;  // Global variable to store SparkNode ID

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) {
    blink_esp32cam_red_led (200, 1200);	// Blink the red LED continuously
  }
} // task_blink_esp32cam_red_led()

void task_udp_heartbeat(void *pvParameter) 
{
  static uint32_t i = 1;
  while (1) {
    blink_esp32cam_flash_led(9, 100);
    blink_esp32cam_flash_led(9, 100);
    ESP_LOGI("UDP heartbeat", "(id %d) %" PRIu32, g_sparknode_id, i++);
    vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
  }
} // task_udp_heartbeat()

static void task_udp_client(void *pvParameters) 
{
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(DESTINATION_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DESTINATION_UDP_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

    while (1) {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", DESTINATION_IP_ADDR, DESTINATION_UDP_PORT);

        while (1) { 
            camera_fb_t *pic = capture_image();
            if (!pic) {
                ESP_LOGE(TAG, "Failed to capture image. Exiting task.");
                break; // Exit the loop if image capture fails
            }      
            esp_err_t result = send_image(sock, &dest_addr, UDP_PACKET_SIZE, pic);
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send image. Exiting task.");
                break; // Exit the loop if sending fails
            }
            esp_camera_fb_return(pic); // Return the frame buffer to be reused again
            vTaskDelay(pdMS_TO_TICKS(20000));
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
} // task_udp_client()

void app_main(void)
{
    get_chip_info(); fflush(stdout);
    //    ESP_ERROR_CHECK(nvs_flash_init());

    initialize_esp32cam_leds(); test_esp32cam_leds();
    if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        indicate_error_esp32cam_red_led();  
        return;
    }
    initialize_wifi_sta(WIFI_SSID, WIFI_PASS);
    blink_esp32cam_flash_led(9, 0);
    initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
    ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
    redirect_log_to_udp();
    
    #ifdef OTA_UPDATE_ENABLED
        ESP_LOGI(TAG, "OTA update enabled");
        check_for_update_and_restart(CHECKSUM_FILE_URL); // If update is not required, 
                                                         // this function will return immediately.
    #endif // OTA_UPDATE_ENABLED
  
    indicate_success_esp32cam_flash_led();
    // int64_t *start_time = malloc(sizeof(int64_t)); 
    // *start_time = esp_timer_get_time();            // For measuring elapsed time
    get_chip_info(); fflush(stdout);

    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    g_sparknode_id = mac_to_id(mac);
    if (g_sparknode_id == 0x00) {
        ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
    } else {
        ESP_LOGI(TAG, "SparkNode ID= %d", g_sparknode_id);
    }      
      
    if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
        ESP_LOGE("app_main", "Failed to create esp32cam blink red led task.");
    }

    if (pdPASS != xTaskCreate(&task_udp_heartbeat, "task_udp_heartbeat", 2048, NULL, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create UDP heartbeat task");
    } 
 
    if(ESP_OK != initialize_camera()) {
        ESP_LOGE(TAG, "Camera initialization failed.");
        indicate_failure_esp32cam_red_led();
        // esp_restart();  // Restart the device if camera initialization fails
    } 

    if (pdPASS != xTaskCreate(&task_udp_client, "udp_client", 8192, NULL, 5, NULL)) {
        ESP_LOGE(TAG, "Failed to create udp client task.");
        indicate_failure_esp32cam_red_led();
        // esp_restart();  // Restart the device if UDP client task creation fails
    }
} // app_main()
