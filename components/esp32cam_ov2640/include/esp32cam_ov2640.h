/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * esp32cam_ov2640.h - ESP32-CAM OV2640 camera driver header
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

#ifndef ESP32CAM_OV2640_H
#define ESP32CAM_OV2640_H

#include <errno.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "lwip/sockets.h" // "lightweight IP" sockets API
#include "esp32cam_leds.h"

// ESP32Cam (AiThinker) pin map
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// Function prototypes
esp_err_t initialize_camera(void);
camera_fb_t *capture_image(void);
esp_err_t send_image(int sock, struct sockaddr_in *dest_addr, 
                           int max_udp_packet_size, camera_fb_t *pic);

#endif // ESP32CAM_OV2640_H
