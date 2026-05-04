/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * fusion_interface.c - AHRS fusion algorithm interface implementation for SynchroSpark
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

#include "fusion_interface.h"
#include "fusion_ahrs.h"
#include <math.h>

static fusion_ahrs_t g_fusion_state;

void fusion_init(float gain)
{
    FusionAHRSInit(&g_fusion_state, gain);
}

void fusion_reset(void)
{
    FusionAHRSReset(&g_fusion_state);
}

void fusion_set_gain(float gain)
{
    g_fusion_state.gain = gain;
}

void fusion_update(float gx_dps, float gy_dps, float gz_dps,
                   float ax, float ay, float az,
                   float mx, float my, float mz,
                   float dt)
{
    if (fabsf(gx_dps) < 0.02f) gx_dps = 0.0f;
    if (fabsf(gy_dps) < 0.02f) gy_dps = 0.0f;
    if (fabsf(gz_dps) < 0.02f) gz_dps = 0.0f;

    FusionAHRSUpdate(&g_fusion_state, gx_dps, gy_dps, gz_dps, ax, ay, az, mx, my, mz, dt);
}

void fusion_update_imu(float gx_dps, float gy_dps, float gz_dps,
                       float ax, float ay, float az,
                       float dt)
{
    if (fabsf(gx_dps) < 0.02f) gx_dps = 0.0f;
    if (fabsf(gy_dps) < 0.02f) gy_dps = 0.0f;
    if (fabsf(gz_dps) < 0.02f) gz_dps = 0.0f;

    FusionAHRSUpdateNoMagnetometer(&g_fusion_state, gx_dps, gy_dps, gz_dps, ax, ay, az, dt);
}

void fusion_seed_imu_yaw(float yaw_deg, float ax, float ay, float az)
{
    FusionAHRSSetYawWithAccel(&g_fusion_state, yaw_deg, ax, ay, az);
}

void fusion_get_orientation(float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    FusionAHRSGetEuler(&g_fusion_state, yaw_deg, pitch_deg, roll_deg);
}

void fusion_get_quaternion(float *w, float *x, float *y, float *z)
{
    FusionAHRSGetQuaternion(&g_fusion_state, w, x, y, z);
}
