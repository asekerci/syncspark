/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * drv8830.c - DRV8830 motor driver implementation for SynchroSpark
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

#include "drv8830.h"
#include "syncspark_config.h"

/*
Register	Address	  Function
--------  ------	  --------
Control	  0x00	    Sets direction + PWM
Fault	    0x01	    Reports faults and errors

The DRV8830 allows 4 modes via the direction bits:
  0x00 = Coast
  0x01 = Forward
  0x02 = Reverse
  0x03 = Brake
The PWM duty cycle is 6 bits (0--63) 

Mode	    Description
----      ----------- 
Forward	  Motor spins in one direction; power applied across terminals (A+ B?).
Reverse	  Motor spins in the other direction (A? B+).
Brake	    Both motor terminals are connected to the same voltage (usually GND), 
          so back-EMF is shorted and motor stops quickly.
Coast	    Motor is disconnected electrically, spins freely until it slows naturally.

Send a command like:
  0x00 register: 0b01xxxxxx (Forward), 0b10xxxxxx (Reverse), where xxxxxx is the PWM value
  Read the 0x01 register to check for errors or overcurrent events.
*/

static esp_err_t drv8830_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, 
                                                                 uint8_t data, int i2c_master_timeout_ms) 
{
  uint8_t write_buffer[2] = {reg_addr, data};
  esp_err_t ret = i2c_master_transmit(dev_handle, write_buffer, sizeof(write_buffer), i2c_master_timeout_ms);
  if (ret != ESP_OK) {
      ESP_LOGE("DRV8830", "Failed to write to DRV8830. Error: %s", esp_err_to_name(ret));
  }
  return ret;
}

void drv8830_set_motor(i2c_master_dev_handle_t dev_handle, uint8_t speed, 
                              bool direction, int i2c_master_timeout_ms) 
{
  uint8_t control_reg = 0x00;

  if (speed > 0x3F) {
      speed = 0x3F; // Limit speed to 6 bits (0-63)
  }
  // Set direction bits (IN1 and IN2)
  if (direction) {
      control_reg |= 0x01; // IN1 = 1, IN2 = 0 (Forward)
  } else {
      control_reg |= 0x02; // IN1 = 0, IN2 = 1 (Reverse)
  }

  // Set speed bits (VSET5:0)
  control_reg |= (speed << 2);

  // Write to the DRV8830 control register
  ESP_ERROR_CHECK(drv8830_write_byte(dev_handle, 0x00, control_reg, i2c_master_timeout_ms));
} // drv8830_set_motor()

static esp_err_t drv8830_read_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, 
                                   uint8_t *data_out, int i2c_master_timeout_ms)
{
  if (!data_out) return ESP_ERR_INVALID_ARG;
  esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data_out, 1, i2c_master_timeout_ms);
  if (ret != ESP_OK) {
    ESP_LOGE("DRV8830", "Failed to read reg 0x%02x. Error: %s", reg_addr, esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t drv8830_read_fault(i2c_master_dev_handle_t dev_handle, uint8_t *fault_reg_out, int i2c_master_timeout_ms)
{
  return drv8830_read_byte(dev_handle, 0x01, fault_reg_out, i2c_master_timeout_ms);
}

void initialize_motor(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms) 
{
  // After a reset, we want to be sure that motor is stopped.
  stop_motor(dev_handle, i2c_master_timeout_ms); 
} // initialize_motor()

void stop_motor(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms) 
{
  drv8830_set_motor(dev_handle, 0x00, false, i2c_master_timeout_ms); // Stop the motor
} // stop_motor()

void stop_motor_with_mode(i2c_master_dev_handle_t dev_handle, drv8830_stop_mode_t mode, int i2c_master_timeout_ms)
{
  /* 
  Modes:
    DRV8830_STOP_COAST: IN1=0, IN2=0 → motor freewheels.
    DRV8830_STOP_BRAKE: IN1=1, IN2=1 → outputs shorted, fast stop.
  How: 
    writes the control register (0x00) with the chosen IN1:IN2 bits and speed=0. No ramp; it takes effect immediately.
  */

  // Mode bits occupy IN1:IN2 positions (bits 1:0). Speed bits must be 0.
  uint8_t control_reg = 0x00;
  if (mode == DRV8830_STOP_BRAKE) {
    control_reg |= 0x03; // IN1=1, IN2=1 -> Brake
  } else {
    control_reg |= 0x00; // IN1=0, IN2=0 -> Coast
  }
  ESP_ERROR_CHECK(drv8830_write_byte(dev_handle, 0x00, control_reg, i2c_master_timeout_ms));
}

static void drv8830_ramp_to(i2c_master_dev_handle_t dev_handle, uint8_t target_speed, bool direction,
                     uint8_t step, uint32_t step_delay_ms, int i2c_master_timeout_ms)
{
  if (step == 0) step = 1;
  if (target_speed > 0x3F) target_speed = 0x3F;
  for (uint8_t s = 0; s <= target_speed; ) {
    drv8830_set_motor(dev_handle, s, direction, i2c_master_timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (target_speed - s < step) break;
    s += step;
  }
  drv8830_set_motor(dev_handle, target_speed, direction, i2c_master_timeout_ms);
}

// Jog the camera motor gently with small ramp and safe stop
/*
If the camera motor struggles, we can:

- Increase the kick strength (+0x14 instead of +0x0C) and/or kick duration (e.g., 180–250 ms).
- Raise the sweep’s base speed in task_camera_sweep from 0x10 to 0x14–0x18.
- Add a “double-kick” (two short pulses with a brief coast).
- Log DRV8830 fault status after the kick to detect current limiting/thermal events.
*/
/*
Optional next tweaks:

- Make sweep params configurable (speed, dwell, pause) via menuconfig or defines.
- Tune the kick (magnitude/duration) or disable it if not needed.
- Add end pauses or bounds-limited sweeping (time-based).
- Read/log DRV8830 fault after kick to catch current limiting.
- Expose jog/sweep controls via UART/UDP commands.
*/
void camera_motor_jog(i2c_master_dev_handle_t cam_motor,
                      uint8_t speed,
                      bool inward,
                      uint32_t duration_ms,
                      int i2c_master_timeout_ms)
{
  if (speed > 0x3F) speed = 0x3F;
  const uint8_t step = 0x03;          // finer steps for precise motion
  const uint32_t step_delay_ms = 50;  // gentle ramp suitable for camera tilt
  
  // Torque boost: apply a stronger short high-duty kick to overcome static friction
  // Choose kick speed above target, capped at max, and hold slightly longer
  uint8_t kick_speed = speed;
  if (kick_speed < 0x3F) {
    uint16_t boosted = (uint16_t) kick_speed + 0x14; // +20/63 (~32%)
    if (boosted < 0x1C) boosted = 0x1C;             // ensure a stronger minimum kick
    if (boosted > 0x3F) boosted = 0x3F;
    kick_speed = (uint8_t) boosted;
  }

  // Fire the kick and give it a short dwell
  drv8830_set_motor(cam_motor, kick_speed, inward, i2c_master_timeout_ms);
  vTaskDelay(pdMS_TO_TICKS(50));

  // If requested speed is below the kick, settle down smoothly to target
  if (speed < kick_speed) {
    for (int s = kick_speed; s > speed; ) {
      uint8_t next = (s - step < (int)s) ? (uint8_t)(s - step) : speed;
      if (next < speed) next = speed;
      drv8830_set_motor(cam_motor, next, inward, i2c_master_timeout_ms);
      vTaskDelay(pdMS_TO_TICKS(30));
      if (s - step <= (int)s) s -= step; else break;
      if (s <= (int)s && s <= (int)s) { /* avoid compiler warnings */ }
      if (s <= (int)s && s <= (int)next) break;
    }
  }

  // Ensure we're at least at target speed (handles case speed >= kick)
  drv8830_set_motor(cam_motor, speed, inward, i2c_master_timeout_ms);

  // Hold
  if (duration_ms > 0) {
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
  }

  // Ramp down
  for (int s = speed; s >= 0; ) {
    drv8830_set_motor(cam_motor, (uint8_t)s, inward, i2c_master_timeout_ms);
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
    if (s < step) break;
    s -= step;
  }

  // Short brake to settle, then coast
  stop_motor_with_mode(cam_motor, DRV8830_STOP_BRAKE, i2c_master_timeout_ms);
  vTaskDelay(pdMS_TO_TICKS(30));
  stop_motor_with_mode(cam_motor, DRV8830_STOP_COAST, i2c_master_timeout_ms);
}

// Drive robot using motor configuration
esp_err_t drive_robot(i2c_master_dev_handle_t left_motor,
                      i2c_master_dev_handle_t right_motor,
                      uint8_t left_speed,
                      uint8_t right_speed,
                      bool forward,
                      uint32_t duration_milliseconds,
                      const void* motor_config_ptr)
{
  const sparknode_motor_config_t* motor_config = (const sparknode_motor_config_t*)motor_config_ptr;
  
  // Apply speed multipliers from configuration
  uint8_t adjusted_left_speed = (uint8_t)(left_speed * motor_config->left_motor_speed_multiplier);
  uint8_t adjusted_right_speed = (uint8_t)(right_speed * motor_config->right_motor_speed_multiplier);
  
  // Clamp to valid range
  if (adjusted_left_speed > 0x3F) adjusted_left_speed = 0x3F;
  if (adjusted_right_speed > 0x3F) adjusted_right_speed = 0x3F;

  // Apply kick-start if enabled
  if (motor_config->use_kick_start) {
    ESP_LOGI("DRV8830", "Kick-start: speed=0x%02X for %ums", 
             motor_config->kick_speed, motor_config->kick_duration_ms);
    
    drv8830_set_motor(left_motor, motor_config->kick_speed, forward, I2C_MASTER_TIMEOUT_MS);
    drv8830_set_motor(right_motor, motor_config->kick_speed, forward, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(motor_config->kick_duration_ms));
  }

  const uint8_t step = 0x05;
  const uint32_t step_delay_ms = 100;

  // Ramp up to target speeds
  uint8_t max_speed = (adjusted_left_speed > adjusted_right_speed) ? adjusted_left_speed : adjusted_right_speed;
  for (uint8_t s = step; s <= max_speed; s += step) {
    uint8_t left_s = (s <= adjusted_left_speed) ? s : adjusted_left_speed;
    uint8_t right_s = (s <= adjusted_right_speed) ? s : adjusted_right_speed;
    
    drv8830_set_motor(left_motor, left_s, forward, I2C_MASTER_TIMEOUT_MS);
    drv8830_set_motor(right_motor, right_s, forward, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
  }

  // Hold at target speeds
  drv8830_set_motor(left_motor, adjusted_left_speed, forward, I2C_MASTER_TIMEOUT_MS);
  drv8830_set_motor(right_motor, adjusted_right_speed, forward, I2C_MASTER_TIMEOUT_MS);
  if (duration_milliseconds > 0) {
    vTaskDelay(pdMS_TO_TICKS(duration_milliseconds));
  }

  // Stop both motors with brake then coast (like other functions)
  stop_motor_with_mode(left_motor, DRV8830_STOP_BRAKE, I2C_MASTER_TIMEOUT_MS);
  stop_motor_with_mode(right_motor, DRV8830_STOP_BRAKE, I2C_MASTER_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(40));
  stop_motor_with_mode(left_motor, DRV8830_STOP_COAST, I2C_MASTER_TIMEOUT_MS);
  stop_motor_with_mode(right_motor, DRV8830_STOP_COAST, I2C_MASTER_TIMEOUT_MS);

  ESP_LOGI("DRV8830", "Drive with config: L=0x%02X(adj from 0x%02X), R=0x%02X(adj from 0x%02X), forward=%d, %"PRIu32"ms",
           adjusted_left_speed, left_speed, adjusted_right_speed, right_speed, forward, duration_milliseconds);
  
  return ESP_OK;
}

// Turn robot using motor configuration  
esp_err_t turn_robot(i2c_master_dev_handle_t left_motor,
                     i2c_master_dev_handle_t right_motor,
                     robot_turn_direction_t direction,
                     uint8_t left_speed,
                     uint8_t right_speed,
                     uint32_t duration_milliseconds,
                     const void* motor_config_ptr)
{
  const sparknode_motor_config_t* motor_config = (const sparknode_motor_config_t*)motor_config_ptr;
  
  // Apply speed multipliers from configuration
  uint8_t adjusted_left_speed = (uint8_t)(left_speed * motor_config->left_motor_speed_multiplier);
  uint8_t adjusted_right_speed = (uint8_t)(right_speed * motor_config->right_motor_speed_multiplier);
  
  // Clamp to valid range
  if (adjusted_left_speed > 0x3F) adjusted_left_speed = 0x3F;
  if (adjusted_right_speed > 0x3F) adjusted_right_speed = 0x3F;

  // Apply kick-start if enabled
  if (motor_config->use_kick_start) {
    ESP_LOGI("DRV8830", "Turn kick-start: speed=0x%02X for %ums", 
             motor_config->kick_speed, motor_config->kick_duration_ms);
    
    bool left_forward = (direction == TURN_RIGHT);
    bool right_forward = (direction == TURN_LEFT);
    
    drv8830_set_motor(left_motor, motor_config->kick_speed, left_forward, I2C_MASTER_TIMEOUT_MS);
    drv8830_set_motor(right_motor, motor_config->kick_speed, right_forward, I2C_MASTER_TIMEOUT_MS);
    vTaskDelay(pdMS_TO_TICKS(motor_config->kick_duration_ms));
  }

  const uint8_t step = 0x05;
  const uint32_t step_delay_ms = 80;

  bool left_forward = (direction == TURN_RIGHT);
  bool right_forward = (direction == TURN_LEFT);

  // Ramp up to target speeds
  uint8_t max_speed = (adjusted_left_speed > adjusted_right_speed) ? adjusted_left_speed : adjusted_right_speed;
  for (uint8_t s = step; s <= max_speed; s += step) {
    uint8_t left_s = (s <= adjusted_left_speed) ? s : adjusted_left_speed;
    uint8_t right_s = (s <= adjusted_right_speed) ? s : adjusted_right_speed;
    
    drv8830_set_motor(left_motor, left_s, left_forward, I2C_MASTER_TIMEOUT_MS);
    drv8830_set_motor(right_motor, right_s, right_forward, I2C_MASTER_TIMEOUT_MS);
    
    vTaskDelay(pdMS_TO_TICKS(step_delay_ms));
  }

  // Hold at target speeds
  drv8830_set_motor(left_motor, adjusted_left_speed, left_forward, I2C_MASTER_TIMEOUT_MS);
  drv8830_set_motor(right_motor, adjusted_right_speed, right_forward, I2C_MASTER_TIMEOUT_MS);

  if (duration_milliseconds > 0) {
    vTaskDelay(pdMS_TO_TICKS(duration_milliseconds));
  }

  // Stop both motors with brake then coast (like other functions)
  stop_motor_with_mode(left_motor, DRV8830_STOP_BRAKE, I2C_MASTER_TIMEOUT_MS);
  stop_motor_with_mode(right_motor, DRV8830_STOP_BRAKE, I2C_MASTER_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(40));
  stop_motor_with_mode(left_motor, DRV8830_STOP_COAST, I2C_MASTER_TIMEOUT_MS);
  stop_motor_with_mode(right_motor, DRV8830_STOP_COAST, I2C_MASTER_TIMEOUT_MS);

  ESP_LOGI("DRV8830", "Turn with config: dir=%s, L=0x%02X(adj from 0x%02X), R=0x%02X(adj from 0x%02X), %"PRIu32"ms",
           (direction == TURN_LEFT) ? "LEFT" : "RIGHT", 
           adjusted_left_speed, left_speed, adjusted_right_speed, right_speed, duration_milliseconds);

  return ESP_OK;
}
