/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * ota_utils.c - Over-the-air update utilities implementation for SynchroSpark
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

#include "ota_utils.h"

/**
 * @brief Calculate the MD5 checksum of a given partition.
 *
 * This function reads the contents of the specified partition and calculates
 * the MD5 checksum of the data. The checksum is stored in the provided
 * `checksum` buffer.
 *
 * @param partition_label The label of the partition to read.
 * @param checksum Buffer to store the resulting MD5 checksum as a hex string.
 * @param total_bytes_written The total number of bytes written to the partition.

 Summary of flow:
  1. Initialize the MD5 context
  2. Read the partition in chunks and update the MD5 context
  3. Finish the MD5 context and free the MD5 context
  4. Calculate the checksum
  5. Print the checksum
*/
void calculate_md5_checksum(const char *partition_label, 
                            char *checksum, size_t total_bytes_written) 
{
    static const char *TAG = "chksum";

    ESP_LOGI(TAG, "Calculating MD5 checksum for partition: %s", partition_label);
    const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, partition_label);
    if (partition == NULL) {
        ESP_LOGE(TAG, "Failed to find partition: %s", partition_label);
        return;
    }

    mbedtls_md5_context md5_ctx;
    mbedtls_md5_init(&md5_ctx);
    mbedtls_md5_starts(&md5_ctx);

    unsigned char buffer[1024];
    size_t read_len;
    size_t offset = 0;
    int total_chunks = 0;
    int total_bytes = 0;
    while (offset < total_bytes_written) {
        read_len = (total_bytes_written - offset) > sizeof(buffer) ? sizeof(buffer) : (total_bytes_written - offset);
        esp_err_t err = esp_partition_read(partition, offset, buffer, read_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read partition at offset %zu: %s", offset, esp_err_to_name(err));
            return;
        }
        total_chunks++;
        total_bytes += read_len;
        printf("\rChunks read: %d, Total bytes read: %d", total_chunks, total_bytes);
        fflush(stdout);
        mbedtls_md5_update(&md5_ctx, buffer, read_len);
        offset += read_len;
        
        if (total_chunks % 10 == 0) {
            blink_esp32cam_flash_led(9, 0); // Flash at every 10 chunks
        }

        // Yield control back to the operating system to prevent watchdog timer 
        // from being triggered
        vTaskDelay(1);
    }

    unsigned char md5sum[16];
    mbedtls_md5_finish(&md5_ctx, md5sum);
    mbedtls_md5_free(&md5_ctx);

    for (int i = 0; i < 16; i++) {
        sprintf(&checksum[i * 2], "%02x", md5sum[i]);
    }
    ESP_LOGI(TAG, "Calculated MD5 checksum: %s", checksum);
} // calculate_md5_checksum()

// read_remote_file_and_write() is responsible for reading a file from a remote server
// and writing it to a specified local destination (an OTA partition or NVS).
//
// Summary of flow:
// if (file_type == FILE_TYPE_TEXT) {
//     1. Opens an HTTP connection to the remote server
//     2. Fetch the headers to get the content length
//     3. Check the HTTP status code
//     4. Read the text file from the remote server
//     5. Write the data to the specified destination (variable or NVS)
//     6. Close the HTTP connection
//     7. Cleanup the HTTP client
// } else if (file_type == FILE_TYPE_BINARY) {
//     1. Opens an HTTP connection to the remote server
//     2. Fetch the headers to get the content length
//     3. Check the HTTP status code
//     4. If file size exceeds partition size, return an error
//     5. Erase the partition
//     6. Read the binary file from the remote server in chunks
//     7. Write the data to the specified destination (OTA partition or NVS)
//     8. Close the HTTP connection
//     9. Cleanup the HTTP client
// } else {
//     1. Return an error for an invalid file type
// }
esp_err_t read_remote_file_and_write(const char *url, file_type_t file_type, 
                                        write_destination_t write_destination, 
                                        size_t *total_bytes_written, char *variable) 
{
    static const char *TAG = "remote_rw";

    *total_bytes_written = 0;

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    ESP_LOGI(TAG, "Opening HTTP connection to %s", url);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "HTTP client fetch headers failed");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    // Check for HTTP status code
    int status_code = esp_http_client_get_status_code(client);
    if (status_code == 404) {
        ESP_LOGE(TAG, "HTTP request failed with status code: %d (Not Found)", status_code);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    } else if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP request failed with status code: %d", status_code);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    if (file_type == FILE_TYPE_TEXT) {
        char buffer[1024];
        int read_len = esp_http_client_read(client, buffer, sizeof(buffer) - 1);
        if (read_len < 0) {
            ESP_LOGE(TAG, "Failed to read text file from HTTP client");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
        buffer[read_len] = '\0';  // Null-terminate the string

        if (write_destination == WRITE_TO_VARIABLE) {
            // Write data to variable
            strncpy(variable, buffer, read_len);
            variable[read_len] = '\0';  // Null-terminate the string
        } else if (write_destination == WRITE_TO_NVS) {
            err = store_data_in_nvs("data", buffer);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to store data in NVS: %s", esp_err_to_name(err));
                esp_http_client_cleanup(client);
                return err;
            }
        } else {
            ESP_LOGE(TAG, "Invalid write destination for text file");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
    } else if (file_type == FILE_TYPE_BINARY) {
        uint8_t buffer[1024];  // Updated chunk size to 1024 bytes
        int total_chunks = 0;
        int read_len;
        const esp_partition_t *ota_partition = NULL;

        if (write_destination == WRITE_TO_OTA_0) {
            ota_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        } else if (write_destination == WRITE_TO_OTA_1) {
            ota_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
        } else if (write_destination == WRITE_TO_NVS) {
            ota_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        } else {
            ESP_LOGE(TAG, "Invalid write destination for binary file");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        if (ota_partition == NULL) {
            ESP_LOGE(TAG, "Failed to find partition");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        // Check if the remote file size fits within the partition size
        if (content_length > ota_partition->size) {
            ESP_LOGE(TAG, "Remote file size (%lu bytes) exceeds partition size (%lu bytes)", (uint32_t)content_length, (uint32_t)ota_partition->size);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        err = esp_partition_erase_range(ota_partition, 0, ota_partition->size);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase partition: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            return err;
        }

        while ((read_len = esp_http_client_read(client, (char *)buffer, sizeof(buffer))) > 0) {
            err = esp_partition_write(ota_partition, total_chunks * sizeof(buffer), buffer, read_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to write to partition: %s", esp_err_to_name(err));
                esp_http_client_cleanup(client);
                return err;
            }
            total_chunks++;
            *total_bytes_written += read_len;
            printf("\rChunks written: %d, Total bytes read: %zu", total_chunks, *total_bytes_written);
            fflush(stdout);
                   
            if (total_chunks % 10 == 0){
                blink_esp32cam_flash_led(9, 0); // Flash at every 10 chunks   
            }
        }

        if (read_len < 0) {
            ESP_LOGE(TAG, "Failed to read data from HTTP client");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        if (*total_bytes_written == 0) {
            ESP_LOGE(TAG, "No data written to partition");
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
        //vTaskDelay(pdMS_TO_TICKS(1000));     // Wait for 1 seconds here 
        //indicate_success();
        ESP_LOGI(TAG, "\nSuccess: Data written to partition");
    } else {
        ESP_LOGE(TAG, "Invalid file type");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return ESP_OK;
} // read_remote_file_and_write()

// check_for_update_and_restart() is responsible for checking for updates and restarting the ESP32.
// Summary of flow:
//  1. Retrieve the stored checksum from the NVS
//  2. Read the remote checksum
//  3. Compare the checksums
//  4. If the checksums match, return ESP_OK
//  5. If the checksums do not match, switch to the factory partition and restart
//  6. Set the boot partition to the factory partition
//  7. Restart the ESP32
//  8. Indicate error if the checksums do not match
//  9. Return ESP_FAIL
esp_err_t check_for_update_and_restart(const char *checksum_file_url) 
{
    static const char *TAG = "chk_update";
    char stored_checksum[33] = {0};
    retrieve_data_from_nvs("checksum", stored_checksum, sizeof(stored_checksum));

    char remote_checksum[33] = {0};
    ESP_LOGI(TAG, "Reading remote checksum...");

    if (read_remote_checksum(checksum_file_url, remote_checksum) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read remote checksum");
        indicate_error_esp32cam_red_led();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Remote checksum --> %s (Length: %d)", remote_checksum, strlen(remote_checksum));

    if (strlen(remote_checksum) != 32) {
        ESP_LOGE(TAG, "Invalid checksum size");
        indicate_error_esp32cam_red_led();
        return ESP_FAIL;
    }

    if (strcmp(remote_checksum, stored_checksum) == 0) {
        ESP_LOGI(TAG, "Checksums match --> %s", stored_checksum);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Checksum mismatch. Switching to factory partition...");
        const esp_partition_t *factory_partition = esp_partition_find_first(
                         ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
        if (factory_partition) {
            if (esp_ota_set_boot_partition(factory_partition) == ESP_OK) {
                ESP_LOGI(TAG, "Set boot partition to 'factory'. Restarting...");
                esp_restart();
            } else {
                ESP_LOGE(TAG, "Failed to set boot partition");
                return ESP_FAIL;
            }
        } else {
            ESP_LOGE(TAG, "Factory partition not found!");
            return ESP_FAIL;
        }
    }
} // check_for_update_and_restart()

esp_err_t read_remote_checksum(const char *url, char *checksum)
{
    static const char *TAG = "read_chksm";

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 10000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    ESP_LOGI(TAG, "Opening HTTP connection to %s", url);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "HTTP client fetch headers failed");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    // Check for HTTP status code
    int status_code = esp_http_client_get_status_code(client);
    if (status_code == 404) {
        ESP_LOGE(TAG, "HTTP request failed with status code: %d (Not Found)", status_code);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    } else if (status_code != 200) {
        ESP_LOGE(TAG, "HTTP request failed with status code: %d", status_code);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    char buffer[33];
    int read_len = esp_http_client_read(client, buffer, sizeof(buffer) - 1);
    if (read_len < 0) {
        ESP_LOGE(TAG, "Failed to read checksum from HTTP client");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    buffer[read_len] = '\0';  // Null-terminate the string

    if (read_len != 32) {
        ESP_LOGE(TAG, "Invalid checksum size");
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    strncpy(checksum, buffer, 32);
    checksum[32] = '\0';  // Null-terminate the string

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return ESP_OK;
} // read_remote_checksum()

void log_elapsed_time(int64_t start_time) 
{
    static const char *TAG = "log_time";
    int64_t end_time = esp_timer_get_time();
    int64_t elapsed_time = end_time - start_time;

    // Convert elapsed time to minutes, seconds, milliseconds, and microseconds
    int64_t elapsed_minutes = elapsed_time / (60 * 1000000);
    elapsed_time %= (60 * 1000000);
    int64_t elapsed_seconds = elapsed_time / 1000000;
    elapsed_time %= 1000000;
    int64_t elapsed_milliseconds = elapsed_time / 1000;
    int64_t elapsed_microseconds = elapsed_time % 1000;   
    ESP_LOGI(TAG, "Elapsed time - %lld min, %lld sec, %lld msec, %lld usec.",
             elapsed_minutes, elapsed_seconds, elapsed_milliseconds, elapsed_microseconds);
} // log_elapsed_time()

void set_next_boot_to_factory(void)
{
  static const char *TAG = "set_next_boot";
  const esp_partition_t *factory_partition = esp_partition_find_first(
                                                      ESP_PARTITION_TYPE_APP,
                                                      ESP_PARTITION_SUBTYPE_APP_FACTORY,
                                                      NULL);
  if (factory_partition != NULL) {
    esp_err_t err = esp_ota_set_boot_partition(factory_partition);
    if (err == ESP_OK) {
      ESP_LOGI(TAG, "Next boot will use factory partition (OTA updater).");
    } else {
      ESP_LOGE(TAG, "Failed to set boot partition to factory: %s", esp_err_to_name(err));
    }
  } else {
    ESP_LOGE(TAG, "Factory partition not found!");
  }
} // set_next_boot_to_factory()
