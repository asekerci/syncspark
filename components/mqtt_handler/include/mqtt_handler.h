/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * mqtt_handler.h - MQTT communication handler for SynchroSpark
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

#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include "mqtt_client.h"
#include "syncspark_config.h"
#include <stdbool.h>

// Function prototypes
void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                                        int32_t event_id, void *event_data);
void publish_position(esp_mqtt_client_handle_t client, const char *hostname, 
                                        int x, int y);

#endif // MQTT_HANDLER_H
