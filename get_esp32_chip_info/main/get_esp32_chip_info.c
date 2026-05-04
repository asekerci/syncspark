/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * get_esp32_chip_info.c - ESP32 chip information utility for SynchroSpark
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
 * We use this program to check the toolchain and to perform 
 * some basic tests on the esp32-cam board.
 * Directly flash this program (via a USB cable) to the factory partition.
 */

#include <stdio.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp32cam_leds.h"
#include "nvs_utils.h"
#include "esp32_chip_info.h"

static int64_t g_start_time;

void log_elapsed_time(int64_t start_time) {
    int64_t end_time = esp_timer_get_time();
    int64_t elapsed_time = end_time - start_time;

    // Convert elapsed time to minutes, seconds, milliseconds, and microseconds
    int64_t elapsed_minutes = elapsed_time / (60 * 1000000);
    elapsed_time %= (60 * 1000000);
    int64_t elapsed_seconds = elapsed_time / 1000000;
    elapsed_time %= 1000000;
    int64_t elapsed_milliseconds = elapsed_time / 1000;
    int64_t elapsed_microseconds = elapsed_time % 1000;

    ESP_LOGI("Elapsed time", "\t%lld min\t%lld sec\t %lld msec\t%lld usec",
             elapsed_minutes, elapsed_seconds, elapsed_milliseconds, elapsed_microseconds);
}

void task_record_time(void *pvParameter)
{
  while (1) {
	log_elapsed_time(g_start_time);
	vTaskDelay(pdMS_TO_TICKS(10000)); // Sleep for 10 seconds
  }
} // task_blink_esp32cam_red_led()

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) {
    blink_esp32cam_red_led(200, 1000);  // Blink the red LED continuously
  }
} // task_blink_esp32cam_red_led()

void app_main(void)
{
  char *TAG = "app_main";

  g_start_time = esp_timer_get_time();
  initialize_esp32cam_leds();
  test_esp32cam_leds();
  if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        indicate_error_esp32cam_red_led();  
        return;
  }
  blink_esp32cam_flash_led(9, 0);
    
  indicate_success_esp32cam_flash_led();
  get_chip_info(); fflush(stdout);

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);

  if (pdPASS  !=  xTaskCreate(&task_record_time, "task_record_time", 2048, NULL, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create record_time task");
  }
  
  if (pdPASS  !=  xTaskCreate(&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }
} // app_main()
