/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * drv5033.c - DRV5033 Hall effect sensor implementation for wheel turn counting
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

// Hall effect sensors implementation for wheel turn counting
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "drv5033.h"

static const char *TAG = "drv5033_hall";

// Global hall sensor data
static hall_sensor_data_t g_sensor_1_data = {0};
static hall_sensor_data_t g_sensor_2_data = {0};
static bool g_initialized = false;
static SemaphoreHandle_t g_hall_mutex = NULL;

// Interrupt handlers
static void sensor_1_isr_handler(void *arg);
static void sensor_2_isr_handler(void *arg);

// Helper functions
static esp_err_t configure_gpio_interrupt(gpio_num_t gpio, gpio_isr_t handler);

esp_err_t initialize_hall_sensors(const hall_sensor_config_t *sensor_1_config,
                                  const hall_sensor_config_t *sensor_2_config) 
{
    if (g_initialized) {
        ESP_LOGW(TAG, "Hall sensors already initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!sensor_1_config || !sensor_2_config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }
    
    // Create mutex for thread safety
    g_hall_mutex = xSemaphoreCreateMutex();
    if (!g_hall_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize data structures
    memset((void *) &g_sensor_1_data, 0, sizeof(hall_sensor_data_t));
    memset((void *) &g_sensor_2_data, 0, sizeof(hall_sensor_data_t));
    memcpy(&g_sensor_1_data.sensor_config, sensor_1_config, sizeof(hall_sensor_config_t));
    memcpy(&g_sensor_2_data.sensor_config, sensor_2_config, sizeof(hall_sensor_config_t));
    
    // Configure sensor 1 GPIO
    esp_err_t ret = configure_gpio_interrupt(sensor_1_config->sensor_gpio, sensor_1_isr_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor 1 GPIO: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Configure sensor 2 GPIO
    ret = configure_gpio_interrupt(sensor_2_config->sensor_gpio, sensor_2_isr_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure sensor 2 GPIO: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Install GPIO interrupt service if not already done
    ret = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);  // Highest priority for time-critical counting
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    // Add ISR handlers
    ret = gpio_isr_handler_add(sensor_1_config->sensor_gpio, sensor_1_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add sensor 1 ISR handler: %s", esp_err_to_name(ret));
        goto cleanup;
    }
    
    ret = gpio_isr_handler_add(sensor_2_config->sensor_gpio, sensor_2_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add sensor 2 ISR handler: %s", esp_err_to_name(ret));
        gpio_isr_handler_remove(sensor_1_config->sensor_gpio);
        goto cleanup;
    }
    
    // Read initial states
    g_sensor_1_data.sensor_state = gpio_get_level(sensor_1_config->sensor_gpio);
    g_sensor_2_data.sensor_state = gpio_get_level(sensor_2_config->sensor_gpio);
    
    g_initialized = true;
    ESP_LOGI(TAG, "Hall sensors initialized successfully");
    ESP_LOGI(TAG, "Sensor 1 GPIO: %d, Magnets per rev: %d", 
             sensor_1_config->sensor_gpio, sensor_1_config->magnets_per_revolution);
    ESP_LOGI(TAG, "Sensor 2 GPIO: %d, Magnets per rev: %d", 
             sensor_2_config->sensor_gpio, sensor_2_config->magnets_per_revolution);
    
    return ESP_OK;
    
cleanup:
    if (g_hall_mutex) {
        vSemaphoreDelete(g_hall_mutex);
        g_hall_mutex = NULL;
    }
    return ret;
}

esp_err_t deinitialize_hall_sensors(void) {
    if (!g_initialized) {
        return ESP_OK;
    }
    
    // Remove ISR handlers
    gpio_isr_handler_remove(g_sensor_1_data.sensor_config.sensor_gpio);
    gpio_isr_handler_remove(g_sensor_2_data.sensor_config.sensor_gpio);
    
    // Clean up mutex
    if (g_hall_mutex) {
        vSemaphoreDelete(g_hall_mutex);
        g_hall_mutex = NULL;
    }
    
    g_initialized = false;
    ESP_LOGI(TAG, "Hall sensors deinitialized");
    
    return ESP_OK;
}

static esp_err_t configure_gpio_interrupt(gpio_num_t gpio, gpio_isr_t handler) 
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,   // Falling edge only - one count per magnet pass
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE  
    };
    
    return gpio_config(&io_conf);
}

static void IRAM_ATTR sensor_1_isr_handler(void *arg) 
{
    // Ultra-minimal ISR - just count, nothing else
    g_sensor_1_data.sensor_count++;
}

static void IRAM_ATTR sensor_2_isr_handler(void *arg) 
{
    // Ultra-minimal ISR - just count, nothing else
    g_sensor_2_data.sensor_count++;
}

uint32_t get_sensor_count(hall_sensor_id_t sensor_id) 
{
    if (!g_initialized) {
        return 0;
    }
    
    switch (sensor_id) {
        case SENSOR_1:
            return g_sensor_1_data.sensor_count;
        case SENSOR_2:
            return g_sensor_2_data.sensor_count;
        default:
            ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
            return 0;
    }
}

void reset_sensor_counts(void) 
{
    if (!g_initialized || !g_hall_mutex) {
        return;
    }
    
    if (xSemaphoreTake(g_hall_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_sensor_1_data.sensor_count = 0;
        g_sensor_1_data.last_sensor_time = 0;
        g_sensor_2_data.sensor_count = 0;
        g_sensor_2_data.last_sensor_time = 0;
        xSemaphoreGive(g_hall_mutex);
        ESP_LOGI(TAG, "Sensor counts reset");
    }
}

float calculate_wheel_revolutions(hall_sensor_id_t sensor_id) 
{
    if (!g_initialized) {
        return 0.0f;
    }
    
    uint32_t count;
    uint8_t magnets_per_rev;
    
    switch (sensor_id) {
        case SENSOR_1:
            count = g_sensor_1_data.sensor_count;
            magnets_per_rev = g_sensor_1_data.sensor_config.magnets_per_revolution;
            break;
        case SENSOR_2:
            count = g_sensor_2_data.sensor_count;
            magnets_per_rev = g_sensor_2_data.sensor_config.magnets_per_revolution;
            break;
        default:
            ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
            return 0.0f;
    }
    
    if (magnets_per_rev == 0) {
        return 0.0f;
    }
    
    return (float) count / (float) magnets_per_rev;
}

float calculate_wheel_angle_degrees(hall_sensor_id_t sensor_id) 
{
    if (!g_initialized) {
        return 0.0f;
    }
    
    uint32_t count;
    uint8_t magnets_per_rev;
    
    switch (sensor_id) {
        case SENSOR_1:
            count = g_sensor_1_data.sensor_count;
            magnets_per_rev = g_sensor_1_data.sensor_config.magnets_per_revolution;
            break;
        case SENSOR_2:
            count = g_sensor_2_data.sensor_count;
            magnets_per_rev = g_sensor_2_data.sensor_config.magnets_per_revolution;
            break;
        default:
            ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
            return 0.0f;
    }
    
    if (magnets_per_rev == 0) {
        return 0.0f;
    }
    
    uint32_t pulses_in_revolution = count % magnets_per_rev;
    float degrees_per_pulse = 360.0f / magnets_per_rev;
    
    return (float)pulses_in_revolution * degrees_per_pulse;
}

int64_t get_time_since_last_pulse_us(hall_sensor_id_t sensor_id) 
{
    if (!g_initialized) {
        return -1;
    }
    
    int64_t last_time;
    switch (sensor_id) {
        case SENSOR_1:
            last_time = g_sensor_1_data.last_sensor_time;
            break;
        case SENSOR_2:
            last_time = g_sensor_2_data.last_sensor_time;
            break;
        default:
            ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
            return -1;
    }
    
    if (last_time == 0) {
        return -1;
    }
    
    return esp_timer_get_time() - last_time;
}

float calculate_wheel_rpm(hall_sensor_id_t sensor_id, uint32_t time_window_ms) 
{
    if (!g_initialized) {
        return 0.0f;
    }
    
    uint32_t count;
    uint8_t magnets_per_rev;
    
    switch (sensor_id) {
        case SENSOR_1:
            count = g_sensor_1_data.sensor_count;
            magnets_per_rev = g_sensor_1_data.sensor_config.magnets_per_revolution;
            break;
        case SENSOR_2:
            count = g_sensor_2_data.sensor_count;
            magnets_per_rev = g_sensor_2_data.sensor_config.magnets_per_revolution;
            break;
        default:
            ESP_LOGE(TAG, "Invalid sensor ID: %d", sensor_id);
            return 0.0f;
    }
    
    if (count == 0 || magnets_per_rev == 0) {
        return 0.0f;
    }
    
    int64_t time_since_last = get_time_since_last_pulse_us(sensor_id);
    if (time_since_last < 0 || time_since_last > (time_window_ms * 1000)) {
        return 0.0f;  // No recent activity
    }
    
    // Simple calculation based on time between pulses
    // More sophisticated implementations could track multiple pulses
    int64_t time_per_pulse_us = time_since_last;
    if (time_per_pulse_us == 0) {
        return 0.0f;
    }
    
    float pulses_per_second = 1000000.0f / time_per_pulse_us;
    float revolutions_per_second = pulses_per_second / magnets_per_rev;
    float rpm = revolutions_per_second * 60.0f;
    
    return rpm;
}

void get_sensor_states(bool *sensor_1_state, bool *sensor_2_state) 
{
    if (sensor_1_state) {
        *sensor_1_state = g_sensor_1_data.sensor_state;
    }
    if (sensor_2_state) {
        *sensor_2_state = g_sensor_2_data.sensor_state;
    }
}

bool wait_for_sensor_2_pulse(uint32_t timeout_ms) 
{
    if (!g_initialized) {
        return false;
    }
    
    uint32_t initial_count = g_sensor_2_data.sensor_count;
    int64_t start_time = esp_timer_get_time();
    int64_t timeout_us = timeout_ms * 1000;
    
    while ((esp_timer_get_time() - start_time) < timeout_us) {
        if (g_sensor_2_data.sensor_count > initial_count) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay to prevent busy waiting
    }
    
    return false;
}

bool is_wheel_moving(hall_sensor_id_t sensor_id, uint32_t threshold_ms) 
{
    int64_t time_since_last = get_time_since_last_pulse_us(sensor_id);
    
    if (time_since_last < 0) {
        return false;  // No pulses detected yet
    }
    
    return time_since_last < (threshold_ms * 1000);
}