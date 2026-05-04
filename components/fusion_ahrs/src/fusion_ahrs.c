/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * FusionAHRS.c - AHRS (Attitude and Heading Reference System) fusion algorithms implementation
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

#include "fusion_ahrs.h"
#include <math.h>

static float clampf_local(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static float wrap360(float angle_deg)
{
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    return angle_deg;
}

static void normalize_vec3(float *x, float *y, float *z)
{
    float norm = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (norm > 0.0f) {
        *x /= norm;
        *y /= norm;
        *z /= norm;
    } else {
        *x = 0.0f;
        *y = 0.0f;
        *z = 0.0f;
    }
}

static void normalize_quaternion(fusion_quaternion_t *q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 0.0f) {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    } else {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

static fusion_quaternion_t quat_conjugate(fusion_quaternion_t q)
{
    fusion_quaternion_t out = {q.w, -q.x, -q.y, -q.z};
    return out;
}

static fusion_quaternion_t quat_multiply(fusion_quaternion_t a, fusion_quaternion_t b)
{
    fusion_quaternion_t out = {
        .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
    return out;
}

static fusion_quaternion_t quat_from_euler(float yaw_rad, float pitch_rad, float roll_rad)
{
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);

    fusion_quaternion_t q = {
        .w = cy * cp * cr + sy * sp * sr,
        .x = cy * cp * sr - sy * sp * cr,
        .y = cy * sp * cr + sy * cp * sr,
        .z = sy * cp * cr - cy * sp * sr,
    };
    normalize_quaternion(&q);
    return q;
}

static fusion_euler_t euler_from_quaternion(fusion_quaternion_t q)
{
    fusion_euler_t out;
    float roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    float pitch = asinf(clampf_local(2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f));
    float yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    out.yaw_deg = wrap360(yaw * (180.0f / (float)M_PI));
    out.pitch_deg = pitch * (180.0f / (float)M_PI);
    out.roll_deg = roll * (180.0f / (float)M_PI);
    return out;
}

static fusion_quaternion_t quat_nlerp(fusion_quaternion_t a, fusion_quaternion_t b, float alpha)
{
    float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (dot < 0.0f) {
        b.w = -b.w;
        b.x = -b.x;
        b.y = -b.y;
        b.z = -b.z;
    }

    fusion_quaternion_t out = {
        .w = (1.0f - alpha) * a.w + alpha * b.w,
        .x = (1.0f - alpha) * a.x + alpha * b.x,
        .y = (1.0f - alpha) * a.y + alpha * b.y,
        .z = (1.0f - alpha) * a.z + alpha * b.z,
    };
    normalize_quaternion(&out);
    return out;
}

static fusion_quaternion_t integrate_gyro(fusion_quaternion_t q, float gx_dps, float gy_dps, float gz_dps, float dt)
{
    const float deg2rad = (float)M_PI / 180.0f;
    fusion_quaternion_t omega = {
        .w = 0.0f,
        .x = gx_dps * deg2rad,
        .y = gy_dps * deg2rad,
        .z = gz_dps * deg2rad,
    };
    fusion_quaternion_t q_dot = quat_multiply(q, omega);
    fusion_quaternion_t out = {
        .w = q.w + 0.5f * q_dot.w * dt,
        .x = q.x + 0.5f * q_dot.x * dt,
        .y = q.y + 0.5f * q_dot.y * dt,
        .z = q.z + 0.5f * q_dot.z * dt,
    };
    normalize_quaternion(&out);
    return out;
}

static fusion_quaternion_t measurement_quaternion(float ax, float ay, float az,
                                                  float mx, float my, float mz)
{
    normalize_vec3(&ax, &ay, &az);
    normalize_vec3(&mx, &my, &mz);

    float roll = atan2f(ay, az);
    float pitch = -atan2f(ax, sqrtf(ay * ay + az * az));

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    float mxh = mx * cp + mz * sp;
    float myh = mx * sr * sp + my * cr - mz * sr * cp;
    float yaw = atan2f(mxh, myh);

    return quat_from_euler(yaw, pitch, roll);
}

static fusion_quaternion_t measurement_quaternion_imu(float ax, float ay, float az,
                                                      float yaw_rad)
{
    normalize_vec3(&ax, &ay, &az);

    float roll = atan2f(ay, az);
    float pitch = -atan2f(ax, sqrtf(ay * ay + az * az));

    return quat_from_euler(yaw_rad, pitch, roll);
}

void FusionAHRSInit(fusion_ahrs_t *state, float gain)
{
    if (!state) {
        return;
    }
    state->gain = gain;
    FusionAHRSReset(state);
}

void FusionAHRSReset(fusion_ahrs_t *state)
{
    if (!state) {
        return;
    }
    state->q.w = 1.0f;
    state->q.x = 0.0f;
    state->q.y = 0.0f;
    state->q.z = 0.0f;
    state->initialized = 0;
}

void FusionAHRSUpdate(fusion_ahrs_t *state,
                      float gx_dps, float gy_dps, float gz_dps,
                      float ax, float ay, float az,
                      float mx, float my, float mz,
                      float dt)
{
    if (!state) {
        return;
    }

    fusion_quaternion_t q_meas = measurement_quaternion(ax, ay, az, mx, my, mz);
    if (!state->initialized) {
        state->q = q_meas;
        state->initialized = 1;
        return;
    }

    fusion_quaternion_t q_gyro = integrate_gyro(state->q, gx_dps, gy_dps, gz_dps, dt);
    float alpha = clampf_local(state->gain * dt, 0.0f, 1.0f);
    state->q = quat_nlerp(q_gyro, q_meas, alpha);
}

void FusionAHRSUpdateNoMagnetometer(fusion_ahrs_t *state,
                                    float gx_dps, float gy_dps, float gz_dps,
                                    float ax, float ay, float az,
                                    float dt)
{
    if (!state) {
        return;
    }

    fusion_quaternion_t q_gyro = state->initialized
        ? integrate_gyro(state->q, gx_dps, gy_dps, gz_dps, dt)
        : state->q;

    fusion_euler_t gyro_euler = euler_from_quaternion(q_gyro);
    float yaw_rad = gyro_euler.yaw_deg * ((float)M_PI / 180.0f);
    fusion_quaternion_t q_meas = measurement_quaternion_imu(ax, ay, az, yaw_rad);

    if (!state->initialized) {
        state->q = q_meas;
        state->initialized = 1;
        return;
    }

    float alpha = clampf_local(state->gain * dt, 0.0f, 1.0f);
    state->q = quat_nlerp(q_gyro, q_meas, alpha);
}

void FusionAHRSSetYawWithAccel(fusion_ahrs_t *state, float yaw_deg,
                               float ax, float ay, float az)
{
    if (!state) {
        return;
    }

    float yaw_rad = yaw_deg * ((float)M_PI / 180.0f);
    state->q = measurement_quaternion_imu(ax, ay, az, yaw_rad);
    state->initialized = 1;
}

void FusionAHRSGetEuler(const fusion_ahrs_t *state, float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    if (!state) {
        return;
    }
    fusion_euler_t e = euler_from_quaternion(state->q);
    if (yaw_deg) {
        *yaw_deg = e.yaw_deg;
    }
    if (pitch_deg) {
        *pitch_deg = e.pitch_deg;
    }
    if (roll_deg) {
        *roll_deg = e.roll_deg;
    }
}

void FusionAHRSGetQuaternion(const fusion_ahrs_t *state, float *w, float *x, float *y, float *z)
{
    if (!state) {
        return;
    }
    if (w) {
        *w = state->q.w;
    }
    if (x) {
        *x = state->q.x;
    }
    if (y) {
        *y = state->q.y;
    }
    if (z) {
        *z = state->q.z;
    }
}
