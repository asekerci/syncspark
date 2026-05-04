/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_ota_updater.c - Over-the-air update manager for SynchroSpark
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

/*
 * This program runs from the factory partition of the ESP32-CAM's flash memory.
 * It is flashed to the device using the `idf.py flash` command (via USB cable connection).
 * See the README.md file for details.
 */

#include <stdio.h>
#include <string.h>
#include "syncspark_config.h"
#include "sys_utils.h"
#include "esp32_chip_info.h"
#include "rgb_led_ring.h"  
#include "ota_utils.h"
#include "wifi_net.h"
#include "esp_sleep.h"

// Create your own file to define these macros:
// #define WIFI_SSID "your wifi ssid here"
// #define WIFI_PASS "your wifi password here"
#include "wifi_credentials.h"

// Contains definitions of DESTINATION_IP_ADDR, DESTINATION_UDP_PORT, 
// LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT, UDP_PACKET_SIZE,
// CHECKSUM_FILE_URL and BINARY_FILE_URL
#include "network_config.h" 

static const char *TAG = "sparknode_ota_updater";

void task_rgb_led_ring(void *pvParameters) 
{
    led_strip_handle_t rgb_led_ring_handle = (led_strip_handle_t) pvParameters;
    
    // Display the De Bruijn sequence for this sparknode progressively and leave it on
    set_leds_from_debruijn_sequence_progressive(rgb_led_ring_handle, g_sparknode_id, 200);
    
    // Task is done - delete itself
    vTaskDelete(NULL);
} // task_rgb_led_ring()

void app_main(void) 
{
    int rgb_led_ring_gpio_pin = RGB_LED_RING_GPIO_PIN;
    int attempt = 0;  // attempt counter
    int64_t start_time = esp_timer_get_time();     // For measuring time in various parts of the code

    get_chip_info(); fflush(stdout);
    if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        log_elapsed_time(start_time);
        indicate_error_esp32cam_red_led();  
        return;
    }
    // vTaskDelay(pdMS_TO_TICKS(500));
    initialize_esp32cam_leds(); // Initialize GPIO pins for built-in LEDs
    test_esp32cam_leds();       // Blink the Flash and red LEDs

    // Obtain the SparkNode ID from the MAC address
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    uint8_t sparknode_id = mac_to_id(mac);
    if (sparknode_id == 0x00) {
        ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
    } else {
        ESP_LOGI(TAG, "SparkNode ID= %d", sparknode_id);
    }

    ESP_LOGI(TAG, "RGB LED ring control pin is %d", rgb_led_ring_gpio_pin);
    initialize_rgb_led_ring(&g_rgb_led_ring_handle, rgb_led_ring_gpio_pin,RGB_LED_RING_LED_COUNT);
    led_strip_clear(g_rgb_led_ring_handle); // Clear the LEDs: Sometimes the ring may not be
                                            // initialized properly and we see some random LEDs are
                                            // lit up. It is better to reset the ring now. 
 
    if (pdPASS != xTaskCreate(&task_rgb_led_ring, "rgb_led_ring task", 4096, g_rgb_led_ring_handle, 5, NULL)) {
        ESP_LOGE("app_main", "Failed to create the rgb_led_ring task.");
    }

    initialize_wifi_sta(WIFI_SSID, WIFI_PASS);
    blink_esp32cam_flash_led(9, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));  
    initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
    ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
    redirect_log_to_udp();

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
        ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
    } else {
        ESP_LOGW(TAG, "Failed to get AP info");
    }

    // Prepare hostname based on SparkNode ID
    char hostname[32];
    if (sparknode_id != 0x00) {
        snprintf(hostname, sizeof(hostname), "sparknode%02d", sparknode_id);
    } else {
        // Fallback to MAC address if SparkNode ID is not available
        snprintf(hostname, sizeof(hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
    }

    // Build the checksum file URL from the base URL and hostname
    // Example: "http://<server>:8000/" + "sparknode01" -> "http://<server>:8000/sparknode01"
    char checksum_file_url[128];
    snprintf(checksum_file_url, sizeof(checksum_file_url), "%s%s.md5", CHECKSUM_FILE_BASE_URL, hostname);
    ESP_LOGI(TAG, "Checksum file URL: %s", checksum_file_url);

    // Build the binary file URL from the base URL and hostname
    // Assuming binaries are named <hostname>.bin under the base URL
    char binary_file_url[128];
    snprintf(binary_file_url, sizeof(binary_file_url), "%s%s.bin", BINARY_FILE_BASE_URL, hostname);
    ESP_LOGI(TAG, "Binary file URL: %s", binary_file_url);

    // Retry logic for fetching the remote checksum
    char remote_checksum[33] = {0};
    for (attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "Attempt %d to fetch the remote MD5 checksum", attempt);
        unsigned int dummy_total_bytes_written = 0; // Dummy variable for the unused argument
        if (read_remote_file_and_write(checksum_file_url, FILE_TYPE_TEXT, WRITE_TO_VARIABLE, 
                                                    &dummy_total_bytes_written, remote_checksum) == ESP_OK) {
            ESP_LOGI(TAG, "Read checksum from remote: %s (Length: %d)", remote_checksum, strlen(remote_checksum));
            if (strlen(remote_checksum) != 32) {
                ESP_LOGE(TAG, "MD5 checksum is not 32 bytes long");
                log_elapsed_time(start_time);
                indicate_error_esp32cam_red_led();
                continue;  // Retry
            }
            blink_esp32cam_flash_led(9, 0); // --> Replacement for flash_tick()
            break;  // Exit loop if successful
        } else {
            ESP_LOGE(TAG, "Failed to fetch the remote checksum, retrying...");
            log_elapsed_time(start_time);
            indicate_error_esp32cam_red_led();  
            continue; // Retry
        } 
        //vTaskDelay(pdMS_TO_TICKS(1000));
    } // End of retry loop for fetching the remote checksum
    
    // Check if the remote checksum is already stored in NVS
    char stored_checksum[33] = {0};
    
    if (attempt > 3) {
        ESP_LOGE(TAG, "Failed to fetch the remote checksum after 3 attempts");
        // Check if there's existing firmware in NVS that we can boot
        if (ESP_OK == retrieve_data_from_nvs("checksum", stored_checksum, sizeof(stored_checksum))) {
            ESP_LOGI(TAG, "Found stored checksum in NVS: %s", stored_checksum);
            ESP_LOGI(TAG, "Network issues detected, booting existing firmware from ota_0 partition...");
            // Set the OTA data partition to boot from the ota_0 partition
            const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
            if (ota_partition != NULL) {
                esp_err_t err = esp_ota_set_boot_partition(ota_partition);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Boot partition set to ota_0, restarting with existing firmware...");
                    indicate_success_esp32cam_flash_led();  // Turn on the Flash LED to indicate success
                    log_elapsed_time(start_time);
                    esp_restart();  // This function does not return
                } else {
                    ESP_LOGE(TAG, "Failed to set the boot partition: %s", esp_err_to_name(err));
                    log_elapsed_time(start_time);
                    indicate_error_esp32cam_red_led();  
                    return;
                }
            } else {
                ESP_LOGE(TAG, "Failed to find the OTA partition");
                log_elapsed_time(start_time);
                indicate_error_esp32cam_red_led();  
                return;
            }
        } else {
            ESP_LOGE(TAG, "No stored checksum found in NVS and unable to fetch remote checksum");
            log_elapsed_time(start_time);
            indicate_failure_esp32cam_red_led();  // Blink the red LED continuously
            return;
        }
    }
    // If we reach here, we successfully read the remote checksum
    ESP_LOGI(TAG, "Remote checksum --> %s", remote_checksum);
    bool firmware_updated = false;
    
    // Check if we already have a stored checksum (might have been retrieved earlier if remote fetch failed)
    bool checksum_already_retrieved = (strlen(stored_checksum) > 0);
    if (!checksum_already_retrieved) {
        retrieve_data_from_nvs("checksum", stored_checksum, sizeof(stored_checksum));
    }
    
    if (strlen(stored_checksum) > 0) {
        ESP_LOGI(TAG, "Stored checksum --> %s", stored_checksum);
        // Compare the remote checksum with the stored checksum
        if (strcmp(remote_checksum, stored_checksum) == 0) {
            ESP_LOGI(TAG, "Remote checksum matches the stored checksum in NVS, no update needed");
            ESP_LOGI(TAG, "Booting existing application from ota_0 partition...");
            // Set the OTA data partition to boot from the ota_0 partition
            const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
            if (ota_partition != NULL) {
                esp_err_t err = esp_ota_set_boot_partition(ota_partition);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Boot partition set to ota_0, restarting...");
                    // vTaskDelay(pdMS_TO_TICKS(1000));     // Wait for 1 second here     
                    indicate_success_esp32cam_flash_led();  // Turn on the Flash LED to indicate success
                    log_elapsed_time(start_time);
                    esp_restart();  // This function does not return
                } else {
                    ESP_LOGE(TAG, "Failed to set the boot partition: %s", esp_err_to_name(err));
                    log_elapsed_time(start_time);
                    indicate_error_esp32cam_red_led();  
                    return;
                }
            } else {
                ESP_LOGE(TAG, "Failed to find the OTA partition");
                log_elapsed_time(start_time);
                indicate_error_esp32cam_red_led();  
                return;
            }
        } else {
            ESP_LOGI(TAG, "Remote checksum does not match the stored checksum in NVS, update needed");
        }
    } else {
        ESP_LOGI(TAG, "No stored checksum found in NVS, firmware update needed");
    }

    // Fetch the remote firmware binary and write to OTA partition
    for (attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "Attempt %d to fetch the firmware binary", attempt);
        unsigned int total_bytes_written = 0;
    if (read_remote_file_and_write(binary_file_url, FILE_TYPE_BINARY, WRITE_TO_OTA_0, &total_bytes_written, NULL) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to fetch the firmware binary, retrying...");
            log_elapsed_time(start_time);
            indicate_error_esp32cam_red_led();
            continue;  // Retry
        }
        //  vTaskDelay(pdMS_TO_TICKS(2000));     

        // Calculate checksum of the downloaded firmware binary
        char calculated_checksum[33];
        calculate_md5_checksum("ota_0", calculated_checksum, total_bytes_written);

        // Print checksums and their lengths before comparison
        ESP_LOGI(TAG, "Remote checksum     --> %s (length: %d)", remote_checksum, strlen(remote_checksum));
        ESP_LOGI(TAG, "Calculated checksum --> %s (length: %d)", calculated_checksum, strlen(calculated_checksum));

        // Compare checksums
        if (strcmp(remote_checksum, calculated_checksum) == 0) { // Checksums match
            ESP_LOGI(TAG, "Checksum verification successful: %s", calculated_checksum);
            firmware_updated = true;
            break;  // Exit the retry loop if checksums match
        } else {
            ESP_LOGE(TAG, "Checksum verification failed. Remote: %s, Calculated: %s", remote_checksum, calculated_checksum);
            log_elapsed_time(start_time);
            indicate_error_esp32cam_red_led();
        }
    } // End of retry loop for fetching the firmware binary
    
    if (attempt > 3) {
        ESP_LOGE(TAG, "Failed to fetch the firmware binary after 3 attempts");
        log_elapsed_time(start_time);
        indicate_failure_esp32cam_red_led();  // Blink the red LED continuously
        return;
    }

    // Only proceed with OTA update if firmware was successfully downloaded and verified
    if (!firmware_updated) {
        ESP_LOGE(TAG, "Firmware was not successfully updated, aborting OTA process");
        log_elapsed_time(start_time);
        indicate_failure_esp32cam_red_led();
        return;
    }
     
    // Write remote checksum to NVS
    if (ESP_OK != store_data_in_nvs("checksum", remote_checksum)) {
        ESP_LOGE(TAG, "Failed to store checksum in NVS");
        log_elapsed_time(start_time);
        indicate_error_esp32cam_red_led();  // Turn on the red LED to indicate error
        return;
    } else { 
        ESP_LOGI(TAG, "Stored remote checksum in NVS: %s", remote_checksum);
        // Set the OTA data partition to boot from the ota_0 partition
        const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
        if (ota_partition != NULL) {
            esp_err_t err = esp_ota_set_boot_partition(ota_partition);
            if (err == ESP_OK ) {
                ESP_LOGI(TAG, "OTA update successful, restarting...");
                vTaskDelay(pdMS_TO_TICKS(1000));     
                indicate_success_esp32cam_flash_led(); 
                log_elapsed_time(start_time);
                esp_restart();  
            } else {
                ESP_LOGE(TAG, "Failed to set the boot partition: %s", esp_err_to_name(err));
                log_elapsed_time(start_time);
                indicate_error_esp32cam_red_led();  
            }
        } else {
            ESP_LOGE(TAG, "Failed to find the OTA partition");
            log_elapsed_time(start_time);
            indicate_error_esp32cam_red_led();  
        }
    } // End of storing remote checksum in NVS
} // app_main()
