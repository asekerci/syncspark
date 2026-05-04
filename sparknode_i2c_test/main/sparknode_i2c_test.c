/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_i2c_test.c - I2C communication testing node for SynchroSpark
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
#include "wifi_net.h"
#include "esp32_chip_info.h"
#include "esp32cam_leds.h"
#include "bmp388.h"
#include "drv8830.h"
#include "icm20948.h"
#include "rgb_led_ring.h"
#include "wifi_credentials.h"
#include "network_config.h"

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

void task_read_bmp388_data(void *pvParameter)
{
  i2c_master_dev_handle_t bmp388_dev_handle = (i2c_master_dev_handle_t) pvParameter;

  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGI("bmp388 stack", "Minimum free stack -> %d words (%d bytes)", 
                                                 uxHighWaterMark, uxHighWaterMark * 4);
  
  while (1) {
    bmp388_read_data(bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
} // task_read_bmp388_data()

/*
void task_read_icm20948_data(void *pvParameter)
{
  i2c_master_dev_handle_t icm20948_dev_handle = (i2c_master_dev_handle_t) pvParameter;
  icm20948_data_t sensor_data;
  icm20948_mag_data_t mag_data;
  
  UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
  ESP_LOGI("icm20948 stack", "Minimum free stack -> %d words (%d bytes)", 
                                                 uxHighWaterMark, uxHighWaterMark * 4);
  
  while (1) {
    //    bmp388_read_data(bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
    icm20948_read_sensor_data(icm20948_dev_handle, &sensor_data);  
    icm20948_read_mag_data(icm20948_dev_handle, &mag_data);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
} // task_read_icm20948v0_data()
*/ 

void task_rgb_led_ring(void *pvParameters) {
  led_strip_handle_t rgb_led_ring_handle = (led_strip_handle_t) pvParameters;
  uint32_t i = 0;

  while (1) {
    // ESP_LOGI(TAG, "(To be added) light up the RGB LED strip");
    ESP_ERROR_CHECK(led_strip_set_pixel(rgb_led_ring_handle, i, 0, 0, 30));
    ESP_ERROR_CHECK(led_strip_refresh(rgb_led_ring_handle));
    i++ ; i %= RGB_LED_RING_LED_COUNT; // Cycle through all LEDs
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(led_strip_clear(rgb_led_ring_handle)); 
  }
} // rgb_led_ring_task()

void app_main (void) 
{
  static const char *TAG = "sparknode_i2c_test";
  
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
  char hostname[32];
  if (g_sparknode_id != 0x00) {
    snprintf(hostname, sizeof(hostname), "sparknode%02d", g_sparknode_id);
  } else {
    // Fallback to MAC address if SparkNode ID is not available
    snprintf(hostname, sizeof(hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }

  initialize_wifi_sta_with_hostname(WIFI_SSID, WIFI_PASS, hostname);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();
  
  indicate_success_esp32cam_flash_led();
  // int64_t start_time = esp_timer_get_time(); 
  get_chip_info(); fflush(stdout);

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }
  
  initialize_rgb_led_ring(&g_rgb_led_ring_handle, RGB_LED_RING_GPIO_PIN, RGB_LED_RING_LED_COUNT);
  led_strip_clear(g_rgb_led_ring_handle); // Clear the LEDs: Sometimes the ring may not be
                                          // initialized properly and we see some random LEDs
                                          // lit up. It is better to reset the ring upon initialization.
  
  initialize_i2c_master(&g_i2c_bus_handle);
  i2c_scan(g_i2c_bus_handle);
  
  /*
  // Add camera motor device to the I2C master bus and obtain device handle.
  i2c_device_config_t cam_motor_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = CAM_MOTOR_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &cam_motor_dev_cfg, &g_cam_motor_dev_handle));
  initialize_motor(g_cam_motor_dev_handle, I2C_MASTER_TIMEOUT_MS);
  */

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
  
  // Add BMP388 device to I2C master bus and obtain device handle.
  i2c_device_config_t bmp388_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BMP388_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &bmp388_dev_cfg, &g_bmp388_dev_handle));
  initialize_bmp388(g_bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
  read_bmp388_chip_id(g_bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);

  /*
  // Verify and initialize ICM20948 sensor
  i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ICM20948_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &icm20948_dev_cfg, &g_icm20948_dev_handle));
  icm20948_verify_device(g_icm20948_dev_handle);
  initialize_icm20948(g_icm20948_dev_handle, I2C_MASTER_TIMEOUT_MS);
	*/ 

  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 4096, NULL, 5, NULL)) {
      ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }
    
  if (pdPASS != xTaskCreate(&task_rgb_led_ring, "rgb_led_ring task", 4096, g_rgb_led_ring_handle, 5, &g_rgb_led_ring_task_handle)) {
    ESP_LOGE("app_main", "Failed to create the rgb_led_ring task.");
  }
  
  /*
  if (pdPASS != xTaskCreate(&task_read_bmp388_data, "task_read_bmp388_data", 4096, g_bmp388_dev_handle, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create read_bmp388_data task");
  }
  */
  
  /* 
  if (pdPASS != xTaskCreate(&task_read_icm20948_data, "task_read_icm20948_data", 2048, g_icm20948_dev_handle, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create read_icm20948_data task");
  }
  */

  /*
  if (pdPASS != xTaskCreate(&task_drive_around, "drive_around task", 4096, NULL, 5, NULL)){
     ESP_LOGE("app_main", "Failed to create drive_around task");
  } 
  */

  // Initialize and start camera platform homing + indexing
 
  /*
  When you test, watch for these log lines (tag: cam_plate):
  * Homing: rotating to find index magnet
  * Homing complete (index=0)
  * Moving X steps to index Y
  * Arrived at index Y If homing never completes, we may need to flip edge polarity 
    (NEG→POSEDGE) or swap which GPIO is home vs ring. Let me know:
  * Which sensor triggers first while turning forward
  * Whether you see extra/spurious index counts Then I can adjust debounce, 
    edge type, or direction. Ready when you are.
  */

  // camera_platform_init(g_cam_motor_dev_handle);
  // camera_platform_start();

  while (1) {
    // i2c_master_initialize();
    // i2c_scan();
    esp_err_t espRc;
    ESP_LOGI(TAG, "Scanning...");

    int addresses[] = {0x61, 0x63, 0x64, 0x68, 0x76, 0x00};
    int num_addresses = sizeof(addresses) / sizeof(addresses[0]);

    for (int i = 0; i < num_addresses; i++) {
      espRc = i2c_master_probe(g_i2c_bus_handle, addresses[i], -1);
      if (espRc == ESP_OK) {
        ESP_LOGI(TAG, "Found device at 0x%02x", addresses[i]);
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "...Done");
    // ESP_ERROR_CHECK(i2c_del_master_bus(g_i2c_bus_handle));
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }

  ESP_LOGI(TAG, "app_main() completed");
}	// app_main()
