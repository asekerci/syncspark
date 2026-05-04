/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_drive_test.c - Motor drive testing node for SynchroSpark
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
#include <string.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "ota_utils.h"
#include "mqtt_handler.h"
#include "wifi_net.h"
#include "esp32_chip_info.h"
#include "esp32cam_leds.h"
#include "bmp388.h"
#include "drv8830.h"
#include "icm20948v0.h"
#include "rgb_led_ring.h"
#include "wifi_credentials.h"
#include "network_config.h"

static uint8_t g_sparknode_id = 0; // Will be set after reading the MAC address
static char g_hostname[32];

void initialize_i2c_master(i2c_master_bus_handle_t *p_i2c_bus_handle)
{
  i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port   = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK (i2c_new_master_bus(&i2c_mst_config, p_i2c_bus_handle));
  ESP_LOGI ("I2C", "I2C master initialization is successful.");
} // initialize_i2c_master()

void i2c_scan(i2c_master_bus_handle_t i2c_bus_handle)
{
  int i;
  int devices_found = 0;
  esp_err_t espRc;
  char *TAG = "I2C Scan";

  ESP_LOGI (TAG, "Starting I2C scan...");
  for (i = 1; i < 127; i++) {
      espRc = i2c_master_probe(i2c_bus_handle, i, -1);
      if (espRc == ESP_OK)	{
	      ESP_LOGI (TAG, "Found device at 0x%02x.", i);
	      devices_found++;  
      }
  }

  if (devices_found == 0) {
      ESP_LOGW (TAG, "No I2C devices detected!");
  } else {
      ESP_LOGI (TAG, "I2C scan complete, %d device(s) found.\n",
		  devices_found);
  }

  // Clean up I2C master bus after scan
  //ESP_LOGI(TAG, "Cleaning up I2C master bus...");
  //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
  //ESP_LOGI(TAG, "I2C master bus cleanup complete");

  // Delay to ensure stability before proceeding
  //vTaskDelay(100 / portTICK_PERIOD_MS);
} // i2c_scan()

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) {
    blink_esp32cam_red_led (200, 1000);	// Blink the red LED continuously
  }
} // task_blink_esp32cam_red_led()

void task_publish_position(void *pvParameter) 
{
  esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameter;
  while (1) {
    // Pick random x and y values for each publish until we have real position data
    // (UHD 4K resolution is 3840x2160)
    int x = (esp_random() % 3840) + 1; // x in [1, 3840]
    int y = (esp_random() % 2160) + 1; // y in [1, 2160]

    publish_position(client, g_hostname, x, y);
    vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds
  }
} // task_publish_position()

void task_drive_around(void *pvParameter) 
{
  const uint8_t speed = 0x15; // Full speed is 0x3F, half speed is 0x1F
  esp_err_t tr;
  
  while (1) {
    ESP_LOGI("motors", "Driving forward...");
    drive_robot(g_left_motor_dev_handle, 
                g_right_motor_dev_handle, 
                speed, 
                /*forward=*/true,  
                /*duration_seconds=*/2, 
                I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // In-place left turn. 
    ESP_LOGI("motors", "Turning left...");
    tr = turn_robot(g_left_motor_dev_handle, 
    						              g_right_motor_dev_handle, 
    						              TURN_LEFT, speed, 
    						              /*duration_seconds=*/1, 
    						              I2C_MASTER_TIMEOUT_MS);
    if (tr != ESP_OK) {
      ESP_LOGE("motors", "turn_robot failed: %s", esp_err_to_name(tr));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("motors", "Driving forward...");
    drive_robot(g_left_motor_dev_handle, 
                g_right_motor_dev_handle, 
                speed, 
                /*forward=*/true,  
                /*duration_seconds=*/2, 
                I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // In-place right turn. 
    ESP_LOGI("motors", "Turning right...");
    tr = turn_robot(g_left_motor_dev_handle, 
    						              g_right_motor_dev_handle, 
    						              TURN_RIGHT, speed, 
    						              /*duration_seconds=*/1, 
    						              I2C_MASTER_TIMEOUT_MS);
    if (tr != ESP_OK) {
      ESP_LOGE("motors", "turn_robot failed: %s", esp_err_to_name(tr));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  } 
} // task_drive_around()

void app_main (void) 
{
  static const char *TAG = "sparknode_drive_test";
  int rgb_led_ring_gpio_pin = RGB_LED_RING_GPIO_PIN;
  
  // We call set_next_boot_to_factory() at the very beginning of app_main()
  // to ensure that the next boot will use the factory partition.
  #ifdef OTA_UPDATE_ENABLED
    ESP_LOGI(TAG, "OTA update enabled");
    set_next_boot_to_factory();
  #endif // OTA_UPDATE_ENABLED
  
  initialize_esp32cam_leds();
  test_esp32cam_leds();
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
  if (g_sparknode_id != 0x00) {
    snprintf(g_hostname, sizeof(g_hostname), "sparknode%02d", g_sparknode_id);
  } else {
    // Fallback to MAC address if SparkNode ID is not available
    snprintf(g_hostname, sizeof(g_hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }

  initialize_wifi_sta_with_hostname(WIFI_SSID, WIFI_PASS, g_hostname);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();
  
  indicate_success_esp32cam_flash_led();
  // int64_t start_time = esp_timer_get_time(); 
  get_chip_info(); fflush(stdout);
  get_reset_reason();

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }

  ESP_LOGI(TAG, "RGB LED ring control pin is %d", rgb_led_ring_gpio_pin);
  initialize_rgb_led_ring(&g_rgb_led_ring_handle, RGB_LED_RING_GPIO_PIN, RGB_LED_RING_LED_COUNT);
  set_leds_from_debruijn_sequence(g_rgb_led_ring_handle, g_sparknode_id);
  
  initialize_i2c_master(&g_i2c_bus_handle);
  i2c_scan(g_i2c_bus_handle);
  
  // Add left motor device to the I2C master bus and obtain device handle.
  i2c_device_config_t left_motor_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = LEFT_MOTOR_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &left_motor_dev_cfg, &g_left_motor_dev_handle));
  initialize_motor(g_left_motor_dev_handle, I2C_MASTER_TIMEOUT_MS);

  // Add right motor device to the I2C master bus and obtain device handle.
  i2c_device_config_t right_motor_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = RIGHT_MOTOR_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &right_motor_dev_cfg, &g_right_motor_dev_handle));
  initialize_motor(g_right_motor_dev_handle,I2C_MASTER_TIMEOUT_MS);

  // Start MQTT client 
  esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = "mqtt://" MQTT_BROKER_IP_ADDR,
  };
  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  // esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, g_hostname);
  esp_mqtt_client_start(client);
  ESP_LOGI(TAG, "MQTT client started");

  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 4096, NULL, 5, NULL)) {
      ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }
  
  if (pdPASS != xTaskCreate(&task_publish_position, "task_publish_position", 4096, (void *)client, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create publish position task");
  }
    
  if (pdPASS != xTaskCreate(&task_drive_around, "drive_around task", 4096, NULL, 5, NULL)){
     ESP_LOGE("app_main", "Failed to create drive_around task");
  } 
}	// app_main()
