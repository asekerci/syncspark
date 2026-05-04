/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * wifi_net.h - WiFi networking utilities for SynchroSpark
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

#ifndef WIFI_NET_H
#define WIFI_NET_H

#include <stdarg.h>
#include <string.h>
#include <sys/param.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

extern const int WIFI_CONNECTED_BIT;

// Function prototypes
void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void initialize_wifi_sta(const char *ssid, const char *password);
void initialize_wifi_sta_with_hostname(const char *ssid, const char *password, const char *hostname);
void initialize_udp_log(const char *log_server_ip_addr, uint16_t log_server_udp_port);
void udp_log_send(const char *tag, esp_log_level_t level, const char *fmt, va_list args);
int  udp_log_vprintf(const char *fmt, va_list args);
void redirect_log_to_udp(void);

#endif // WIFI_NET_H