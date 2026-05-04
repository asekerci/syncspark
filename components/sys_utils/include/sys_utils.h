/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sys_utils.h - System utilities for SynchroSpark
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

#ifndef SYS_UTILS_H
#define SYS_UTILS_H

#include <stdint.h>
#include <stddef.h>
#include "esp_mac.h"

// Function prototypes:
uint8_t mac_to_id(const uint8_t mac[6]);
uint8_t get_debruijn_sequence_index(uint8_t sparknode_id);
void get_reset_reason(void);

#endif // SYS_UTILS_H
