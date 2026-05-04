/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * glow_ledring.c - LED ring glow effects for SynchroSpark
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

#include <stdio.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "esp32_chip_info.h"
#include "esp32cam_leds.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "nvs_utils.h"
#include "rgb_led_ring.h"  

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

// static uint8_t g_sparknode_id = 0; // Will be set after reading the MAC address
// static led_strip_handle_t g_rgb_led_ring_handle = NULL;
// static TaskHandle_t rgb_led_ring_task_handle = NULL;

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) { // Blink the red LED continuously
    blink_esp32cam_red_led (200, 1000);
  }
} // task_blink_esp32cam_red_led()

void task_rgb_led_ring(void *pvParameters) {
    led_strip_handle_t rgb_led_ring_handle = (led_strip_handle_t) pvParameters;
    
    while (1) {
        // Display the De Bruijn sequence for this sparknode
        set_leds_from_debruijn_sequence(rgb_led_ring_handle, g_sparknode_id);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Show pattern for 5 seconds
        
        // Clear LEDs
        ESP_ERROR_CHECK(led_strip_clear(rgb_led_ring_handle));
        vTaskDelay(pdMS_TO_TICKS(1000)); // Off for 1 second
    }
} // rgb_led_ring_task()

void app_main (void) 
{
  char *TAG = "glow_ledring";
  int rgb_led_ring_gpio_pin = RGB_LED_RING_GPIO_PIN;
  
  initialize_esp32cam_leds(); test_esp32cam_leds();       
  if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        indicate_error_esp32cam_red_led();  
        return;
  }
  indicate_success_esp32cam_flash_led();
  //int64_t *start_time = malloc(sizeof(int64_t)); 
  //*start_time = esp_timer_get_time();            // For measuring elapsed time
  get_chip_info(); fflush(stdout);
  get_reset_reason();
  
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  g_sparknode_id = mac_to_id(mac);
  if (g_sparknode_id == 0x00) {
    ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
  } else {
    ESP_LOGI(TAG, "SparkNode ID is %d", g_sparknode_id);
  }      
  ESP_LOGI(TAG, "RGB LED ring control pin is %d", rgb_led_ring_gpio_pin);
  initialize_rgb_led_ring(&g_rgb_led_ring_handle, rgb_led_ring_gpio_pin,RGB_LED_RING_LED_COUNT);
  led_strip_clear(g_rgb_led_ring_handle); 
    
  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }

  if (pdPASS != xTaskCreate(&task_rgb_led_ring, "rgb_led_ring task", 4096, g_rgb_led_ring_handle, 5, NULL)) {
    ESP_LOGE("app_main", "Failed to create the rgb_led_ring task.");
  }
}	// app_main()

