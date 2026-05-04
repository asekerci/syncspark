/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * FusionAHRS.h - AHRS (Attitude and Heading Reference System) fusion algorithms
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

#ifndef FUSION_AHRS_H
#define FUSION_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float w;
    float x;
    float y;
    float z;
} fusion_quaternion_t;

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} fusion_euler_t;

typedef struct {
    fusion_quaternion_t q;
    float gain;
    int initialized;
} fusion_ahrs_t;

void FusionAHRSInit(fusion_ahrs_t *state, float gain);
void FusionAHRSReset(fusion_ahrs_t *state);
void FusionAHRSUpdate(fusion_ahrs_t *state,
                      float gx_dps, float gy_dps, float gz_dps,
                      float ax, float ay, float az,
                      float mx, float my, float mz,
                      float dt);
void FusionAHRSUpdateNoMagnetometer(fusion_ahrs_t *state,
                                    float gx_dps, float gy_dps, float gz_dps,
                                    float ax, float ay, float az,
                                    float dt);
void FusionAHRSSetYawWithAccel(fusion_ahrs_t *state, float yaw_deg,
                               float ax, float ay, float az);
void FusionAHRSGetEuler(const fusion_ahrs_t *state, float *yaw_deg, float *pitch_deg, float *roll_deg);
void FusionAHRSGetQuaternion(const fusion_ahrs_t *state, float *w, float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif

#endif
