/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32cam_ov2640.c - ESP32-CAM OV2640 camera driver implementation
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

#include "esp32cam_ov2640.h"

esp_err_t initialize_camera(void)
{
    const char *TAG = "initialize_camera";
    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
    
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,
    
        // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
    
        .pixel_format = PIXFORMAT_JPEG,     // Options: YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = FRAMESIZE_QVGA,       // QQVGA-UXGA, For ESP32, do not use sizes above QVGA
                                            // when not JPEG. 
                                            // The performance of the ESP32-S series has improved a 
                                            // lot, but JPEG mode always gives better frame rates.
                                            // QVGA is 320 X 240 pixels big.
    
        .jpeg_quality = 10,                 // 0-63, for OV series camera sensors, lower number 
                                            // means higher quality.
        
        .fb_count = 2,                      // When jpeg mode is used, if fb_count is >1, the 
                                            // driver will work in continuous mode.

        .fb_location = CAMERA_FB_IN_PSRAM,  // Other option(s): CAMERA_FB_IN_DRAM
        .grab_mode = CAMERA_GRAB_LATEST     // Other option(s): CAMERA_GRAB_WHEN_EMPTY
    };
    
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera initialization failed.");
        return err;
    }
    return ESP_OK;
}

/**
 * @brief Captures an image using the ESP32 camera
 * @return Pointer to the captured image (camera_fb_t) or NULL if capture fails
 */
camera_fb_t *capture_image(void) {
    const char *TAG = "capture_image";
    
    // Check available memory before capture
    size_t free_heap = esp_get_free_heap_size();
    if (free_heap < 50000) {  // Less than 50KB free
        ESP_LOGW(TAG, "Low memory: %zu bytes free", free_heap);
    }
    
    camera_fb_t *pic = esp_camera_fb_get();
    if (!pic) {
        ESP_LOGE(TAG, "Failed to capture the image.");
        return NULL;
    }
    ESP_LOGI(TAG, "Captured an image of size: %zu bytes.", pic->len);
    ESP_LOGI(TAG, "Frame buffer address: %p", (void *)pic);
    ESP_LOGI(TAG, "Capture details:");
    ESP_LOGI(TAG, "  Buffer address: %p", (void *)pic);
    ESP_LOGI(TAG, "  Buffer len: %zu", pic->len);
    ESP_LOGI(TAG, "  Width: %d, Height: %d", pic->width, pic->height);
    ESP_LOGI(TAG, "  Format: %d", pic->format);
    indicate_success_esp32cam_flash_led(); 
    return pic;
}

/**
 * @brief Sends the captured image over UDP
 * @param sock The UDP socket
 * @param dest_addr The destination address
 * @param pic Pointer to the captured image (camera_fb_t)
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t send_image(int sock, struct sockaddr_in *dest_addr, int max_udp_packet_size, camera_fb_t *pic) {
    const char *TAG = "send_image";
    size_t packet_size, bytes_sent = 0;

    while (bytes_sent < pic->len) {
        packet_size = (pic->len - bytes_sent) > max_udp_packet_size ? max_udp_packet_size : (pic->len - bytes_sent);
        ESP_LOGI(TAG, "Sending %zu bytes (UDP packet_size)", packet_size);
        int err = sendto(sock, pic->buf + bytes_sent, packet_size, 0, (struct sockaddr *)dest_addr, sizeof(*dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            return ESP_FAIL;
        }
        bytes_sent += packet_size;
    }
    ESP_LOGI(TAG, "Image was sent successfully (%zu bytes).", pic->len);
    return ESP_OK;
}

