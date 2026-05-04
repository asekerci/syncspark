/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * wifi_net.c - WiFi networking utilities implementation for SynchroSpark
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

#include "wifi_net.h"
#include "esp_netif.h"

static const char *TAG = "wifi_net";
static EventGroupHandle_t wifi_event_group;
static int udp_socket = -1;
static struct sockaddr_in log_server_addr;
static bool udp_log_initialized = false;
static char log_server_ip_str[16];
static uint16_t log_server_port;

const int WIFI_CONNECTED_BIT = BIT0;

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        // ESP_LOGW(TAG, "WiFi disconnected, attempting to reconnect...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        // Close UDP socket on disconnect to force recreation
        if (udp_socket >= 0) {
            lwip_close(udp_socket);
            udp_socket = -1;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        // Reinitialize UDP logging after getting IP
        if (udp_log_initialized) {
            initialize_udp_log(log_server_ip_str, log_server_port);
        }
    }
}

void initialize_wifi_sta(const char *wifi_ssid, const char *wifi_password) {
    initialize_wifi_sta_with_hostname(wifi_ssid, wifi_password, NULL);
}

void initialize_wifi_sta_with_hostname(const char *wifi_ssid, const char *wifi_password, const char *hostname) {
    ESP_LOGI(TAG, "Initializing Wi-Fi in station mode...");
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    
    // Set hostname if provided
    if (hostname != NULL && netif != NULL) {
        esp_err_t ret = esp_netif_set_hostname(netif, hostname);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Hostname set to: %s", hostname);
        } else {
            ESP_LOGE(TAG, "Failed to set hostname: %s", esp_err_to_name(ret));
        }
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_sta initalization is complete, waiting for connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

void initialize_udp_log(const char *log_server_ip_addr, uint16_t log_server_udp_port)
{
    // Store parameters for potential reinitializations
    strncpy(log_server_ip_str, log_server_ip_addr, sizeof(log_server_ip_str) - 1);
    log_server_ip_str[sizeof(log_server_ip_str) - 1] = '\0';
    log_server_port = log_server_udp_port;
    udp_log_initialized = true;

    // Close existing socket if any
    if (udp_socket >= 0) {
        lwip_close(udp_socket);
        udp_socket = -1;
    }

    udp_socket = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket < 0) {
    	// Don't use ESP_LOGE() here to avoid (indirect) recursive function calls 
        // ESP_LOGE("UDP_LOG", "Failed to create socket: errno %d", errno);
        return;
    }

    // Set socket options for better reliability
    int optval = 1;
    lwip_setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    
    // Set send timeout
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    lwip_setsockopt(udp_socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    memset(&log_server_addr, 0, sizeof(log_server_addr));
    log_server_addr.sin_family = AF_INET;
    log_server_addr.sin_port = htons(log_server_udp_port);
    log_server_addr.sin_addr.s_addr = inet_addr(log_server_ip_addr);

    ESP_LOGI("UDP_LOG", "UDP logging initialized to %s:%d", log_server_ip_addr, log_server_udp_port);
}

void udp_log_send(const char *tag, esp_log_level_t level, const char *fmt, va_list args)
{
    // Check if WiFi is connected
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (!(bits & WIFI_CONNECTED_BIT)) {
        return; // WiFi not connected, skip sending
    }

    // Check if socket is valid
    if (udp_socket < 0) {
        // Try to reinitialize if we have the parameters
        if (udp_log_initialized) {
            initialize_udp_log(log_server_ip_str, log_server_port);
            if (udp_socket < 0) {
                return; // Still failed, give up
            }
        } else {
            return;
        }
    }

    char log_buffer[256];
    int len = vsnprintf(log_buffer, sizeof(log_buffer), fmt, args);
    if (len > 0 && len < sizeof(log_buffer)) {
        int sent = lwip_sendto(udp_socket, log_buffer, len, 0,
                    (struct sockaddr *)&log_server_addr, sizeof(log_server_addr));
        if (sent < 0) {
            // Socket might have become invalid, close it
            lwip_close(udp_socket);
            udp_socket = -1;
            // Don't use ESP_LOGW()  here to avoid (indirect) recursive 
            // logging function calls!
            // ESP_LOGW("UDP_LOG", "Failed to send UDP log, socket closed for reinit");
        }
    }
}

int udp_log_vprintf(const char *fmt, va_list args) {
    udp_log_send("UDP_LOG", ESP_LOG_INFO, fmt, args);
    return vprintf(fmt, args);
}

void redirect_log_to_udp(void)
{
    esp_log_set_vprintf(udp_log_vprintf);
}

void cleanup_udp_log(void)
{
    if (udp_socket >= 0) {
        lwip_close(udp_socket);
        udp_socket = -1;
    }
    udp_log_initialized = false;
}
