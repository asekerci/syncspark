/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * fusion_interface.h - AHRS fusion algorithm interface for SynchroSpark
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

#ifndef FUSION_INTERFACE_H
#define FUSION_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

void fusion_init(float gain);
void fusion_reset(void);
void fusion_set_gain(float gain);
void fusion_update(float gx_dps, float gy_dps, float gz_dps,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float dt);
void fusion_update_imu(float gx_dps, float gy_dps, float gz_dps,
                       float ax, float ay, float az,
                       float dt);
void fusion_seed_imu_yaw(float yaw_deg, float ax, float ay, float az);
void fusion_get_orientation(float *yaw_deg, float *pitch_deg, float *roll_deg);
void fusion_get_quaternion(float *w, float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif
