/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * mqtt_handler.c - MQTT communication handler implementation for SynchroSpark
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

#include "mqtt_handler.h"
#include "drv8830.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "icm20948.h"
#include "mqtt_client.h"
#include "syncspark_config.h"
#include <ctype.h>
#include <math.h> // <-- for fabs()
#include <stdio.h>
#include <string.h>

// Default motion parameters
#ifndef ROBOT_DRIVE_SPEED
#define ROBOT_DRIVE_SPEED 0x20 // 0..0x3F
#endif
#ifndef ROBOT_I2C_TIMEOUT_MS
#define ROBOT_I2C_TIMEOUT_MS 1000
#endif
#ifndef MAG_CAPTURE_DEFAULT_SAMPLES
#define MAG_CAPTURE_DEFAULT_SAMPLES 500
#endif
#ifndef MAG_CAPTURE_MIN_SAMPLES
#define MAG_CAPTURE_MIN_SAMPLES 100
#endif
#ifndef MAG_CAPTURE_MAX_SAMPLES
#define MAG_CAPTURE_MAX_SAMPLES 1200
#endif
#ifndef MAG_CAPTURE_DEFAULT_DELAY_MS
#define MAG_CAPTURE_DEFAULT_DELAY_MS 50
#endif

// Device handles are provided by syncspark_config component
extern i2c_master_dev_handle_t g_left_motor_dev_handle;
extern i2c_master_dev_handle_t g_right_motor_dev_handle;
extern i2c_master_dev_handle_t g_icm20948_dev_handle;

// Add these externs for sensor data
extern icm20948_mag_data_t g_mag_data;
extern icm20948_data_t g_sensor_data;

typedef struct {
  bool forward;                   // for drive
  uint8_t speed;                  // speed value 0-63
  uint32_t duration_milliseconds; // hold time in milliseconds
} drive_task_args_t;

typedef struct {
  robot_turn_direction_t dir;     // left or right
  uint8_t speed;                  // speed value 0-63
  uint32_t duration_milliseconds; // hold time in milliseconds
} turn_task_args_t;

static void task_drive_robot(void *pv)
{
  drive_task_args_t *args = (drive_task_args_t *)pv;

  esp_err_t result = drive_robot(g_left_motor_dev_handle, g_right_motor_dev_handle, args->speed, args->speed,
                                 args->forward, args->duration_milliseconds, &g_motor_config);
  if (result != ESP_OK) {
    ESP_LOGE("MQTT", "Drive robot with config failed: %s", esp_err_to_name(result));
  }
  vPortFree(args);
  vTaskDelete(NULL);
} // task_drive_robot()

static void task_turn_robot(void *pv)
{
  turn_task_args_t *args = (turn_task_args_t *)pv;

  esp_err_t result = turn_robot(g_left_motor_dev_handle, g_right_motor_dev_handle, args->dir, args->speed, args->speed,
                                args->duration_milliseconds, &g_motor_config);
  if (result != ESP_OK) {
    ESP_LOGE("MQTT", "Turn robot with config failed: %s", esp_err_to_name(result));
  }
  vPortFree(args);
  vTaskDelete(NULL);
} // task_turn_robot()

static void start_drive(bool forward, uint8_t speed, uint32_t milliseconds)
{
  drive_task_args_t *args = pvPortMalloc(sizeof(*args));
  if (!args) {
    ESP_LOGE("MQTT", "Failed to allocate drive task args");
    return;
  }
  args->forward = forward;
  args->speed = speed;
  args->duration_milliseconds = milliseconds;
  if (xTaskCreate(task_drive_robot, forward ? "drive_fwd" : "drive_rev", 3072, args, 6, NULL) != pdPASS) {
    ESP_LOGE("MQTT", "Failed to create drive task");
    vPortFree(args);
  }
} // start_drive()

static void start_turn(robot_turn_direction_t dir, uint8_t speed, uint32_t milliseconds)
{
  turn_task_args_t *args = pvPortMalloc(sizeof(*args));
  if (!args) {
    ESP_LOGE("MQTT", "Failed to allocate turn task args");
    return;
  }
  args->dir = dir;
  args->speed = speed;
  args->duration_milliseconds = milliseconds;
  if (xTaskCreate(task_turn_robot, dir == TURN_LEFT ? "turn_left" : "turn_right", 3072, args, 6, NULL) != pdPASS) {
    ESP_LOGE("MQTT", "Failed to create turn task");
    vPortFree(args);
  }
} // start_turn()

// Immediately stop both drive motors (brake briefly, then coast)
static void stop_all_motors(void)
{
  stop_motor_with_mode(g_left_motor_dev_handle, DRV8830_STOP_BRAKE, ROBOT_I2C_TIMEOUT_MS);
  stop_motor_with_mode(g_right_motor_dev_handle, DRV8830_STOP_BRAKE, ROBOT_I2C_TIMEOUT_MS);
  vTaskDelay(pdMS_TO_TICKS(40));
  stop_motor_with_mode(g_left_motor_dev_handle, DRV8830_STOP_COAST, ROBOT_I2C_TIMEOUT_MS);
  stop_motor_with_mode(g_right_motor_dev_handle, DRV8830_STOP_COAST, ROBOT_I2C_TIMEOUT_MS);
} // stop_all_motors()

static void publish_status(esp_mqtt_client_handle_t client, const char *topic_status, const char *hostname,
                           const char *msg)
{
  if (!msg) {
    return;
  }

  if (!hostname || hostname[0] == '\0') {
    esp_mqtt_client_publish(client, topic_status, msg, 0, 0, 0);
    return;
  }

  size_t hostname_len = strlen(hostname);
  if (strncmp(msg, hostname, hostname_len) == 0 && (msg[hostname_len] == '\0' || msg[hostname_len] == ' ')) {
    esp_mqtt_client_publish(client, topic_status, msg, 0, 0, 0);
    return;
  }

  char prefixed_msg[256];
  snprintf(prefixed_msg, sizeof(prefixed_msg), "%s %s", hostname, msg);
  esp_mqtt_client_publish(client, topic_status, prefixed_msg, 0, 0, 0);
}

static bool ensure_mag_available(esp_mqtt_event_handle_t event, const char *topic_status, const char *hostname,
                                 const char *cmd_label)
{
  if (!g_icm20948_dev_handle) {
    char msg[96];
    snprintf(msg, sizeof(msg), "%s failed: IMU handle not initialized", cmd_label);
    publish_status(event->client, topic_status, hostname, msg);
    return false;
  }

  if (!g_mag_ready) {
    char msg[112];
    snprintf(msg, sizeof(msg), "%s failed: magnetometer not initialized/ready", cmd_label);
    publish_status(event->client, topic_status, hostname, msg);
    return false;
  }

  return true;
}

static const char *sensor_fusion_mode_name(sensor_stream_mode_t mode)
{
  switch (mode) {
  case MODE_ACCEL_GYRO:
    return "accel_gyro";
  case MODE_MAG:
    return "mag";
  case MODE_MAG_ACCEL:
    return "mag_accel";
  case MODE_MAG_ACCEL_GYRO:
  default:
    return "mag_accel_gyro";
  }
}

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  static const char *TAG = "mqtt_handler";
  esp_mqtt_event_handle_t event = event_data;
  const char *hostname = (const char *)handler_args;
  char topic_cmd[64], topic_status[64];

  snprintf(topic_cmd, sizeof(topic_cmd), "arena/%s/cmd", hostname);
  snprintf(topic_status, sizeof(topic_status), "arena/%s/status", hostname);
  switch ((esp_mqtt_event_id_t)event_id) {
  case MQTT_EVENT_CONNECTED: {
    ESP_LOGI(TAG, "Connected to broker");
    esp_mqtt_client_subscribe(event->client, topic_cmd, 0);
    esp_mqtt_client_subscribe(event->client, "arena/all/cmd", 0);
    esp_mqtt_client_subscribe(event->client, "arena/all/position", 0);
    char online_msg[80];
    snprintf(online_msg, sizeof(online_msg), "%s is online", hostname);
    publish_status(event->client, topic_status, hostname, online_msg);
    break;
  }
  case MQTT_EVENT_DATA: {
    ESP_LOGI(TAG, "Got message on topic: %.*s -> %.*s", event->topic_len, event->topic, event->data_len, event->data);
    // Copy topic and payload to null-terminated buffers for parsing/logging.
    char topic_in[128];
    size_t topic_n = (event->topic_len < (int)sizeof(topic_in) - 1) ? (size_t)event->topic_len : sizeof(topic_in) - 1;
    memcpy(topic_in, event->topic, topic_n);
    topic_in[topic_n] = '\0';

    char msg[384];
    size_t n = (event->data_len < sizeof(msg) - 1) ? event->data_len : sizeof(msg) - 1;
    memcpy(msg, event->data, n);
    msg[n] = '\0';

    if (event->data_len >= (int)sizeof(msg)) {
      ESP_LOGW(TAG, "Incoming command truncated: len=%d max=%u", event->data_len, (unsigned)(sizeof(msg) - 1));
      publish_status(event->client, topic_status, hostname, "Command too long and was truncated");
    }

    ESP_LOGI(TAG, "Received command string: '%s'", msg);

    if (strcmp(topic_in, topic_cmd) == 0 || strcmp(topic_in, "arena/all/cmd") == 0) {
      publish_status(event->client, topic_status, hostname, msg);
    }

    // Basic commands
    if (strcasecmp(msg, "reboot") == 0) {
      ESP_LOGW(TAG, "Reboot command received via MQTT. Rebooting now!");
      char reboot_msg[80];
      snprintf(reboot_msg, sizeof(reboot_msg), "%s is rebooting", hostname);
      publish_status(event->client, topic_status, hostname, reboot_msg);
      vTaskDelay(pdMS_TO_TICKS(100)); // Give time for log flush
      esp_restart();
      break;
    }

    // stop
    if (strcasecmp(msg, "stop") == 0) {
      ESP_LOGI(TAG, "Stop command received");
      stop_all_motors();
      publish_status(event->client, topic_status, hostname, "stopped");
      break;
    }

    // drive forward/reverse <speed> <duration>
    // Accept forms: "drive forward 32 3000" or "drive reverse 20 2500" (speed 0-63, duration in milliseconds)
    {
      unsigned speed = 0, milliseconds = 0;
      if (strncasecmp(msg, "drive forward", 13) == 0) {
        const char *p = msg + 13;
        while (*p && isspace((unsigned char)*p))
          p++;
        if (sscanf(p, "%u %u", &speed, &milliseconds) == 2 && speed <= 63 && milliseconds > 0) {
          char info[80];
          snprintf(info, sizeof(info), "driving forward speed=%u %ums", speed, milliseconds);
          publish_status(event->client, topic_status, hostname, info);
          start_drive(true, (uint8_t)speed, milliseconds);
          break;
        }
      } else if (strncasecmp(msg, "drive reverse", 13) == 0) {
        const char *p = msg + 13;
        while (*p && isspace((unsigned char)*p))
          p++;
        if (sscanf(p, "%u %u", &speed, &milliseconds) == 2 && speed <= 63 && milliseconds > 0) {
          char info[80];
          snprintf(info, sizeof(info), "driving reverse speed=%u %ums", speed, milliseconds);
          publish_status(event->client, topic_status, hostname, info);
          start_drive(false, (uint8_t)speed, milliseconds);
          break;
        }
      }
    }

    // turn left/right <speed> <duration>
    {
      unsigned speed = 0, milliseconds = 0;
      if (strncasecmp(msg, "turn left", 9) == 0) {
        const char *p = msg + 9;
        while (*p && isspace((unsigned char)*p))
          p++;
        if (sscanf(p, "%u %u", &speed, &milliseconds) == 2 && speed <= 63 && milliseconds > 0) {
          char info[80];
          snprintf(info, sizeof(info), "turning left speed=%u %ums", speed, milliseconds);
          publish_status(event->client, topic_status, hostname, info);
          start_turn(TURN_LEFT, (uint8_t)speed, milliseconds);
          break;
        }
      } else if (strncasecmp(msg, "turn right", 10) == 0) {
        const char *p = msg + 10;
        while (*p && isspace((unsigned char)*p))
          p++;
        if (sscanf(p, "%u %u", &speed, &milliseconds) == 2 && speed <= 63 && milliseconds > 0) {
          char info[80];
          snprintf(info, sizeof(info), "turning right speed=%u %ums", speed, milliseconds);
          publish_status(event->client, topic_status, hostname, info);
          start_turn(TURN_RIGHT, (uint8_t)speed, milliseconds);
          break;
        }
      }
    }
    // show calibrations - show current active mag and gyro calibrations
    if (strcasecmp(msg, "show calibrations") == 0) {
      char line[192];

      snprintf(line, sizeof(line), "mag: bias=[%.3f %.3f %.3f] ver=%u ready=%s", g_mag_bias.hard_iron_bias[0],
               g_mag_bias.hard_iron_bias[1], g_mag_bias.hard_iron_bias[2], (unsigned)g_mag_bias.version,
               g_mag_ready ? "yes" : "no");
      publish_status(event->client, topic_status, hostname, line);

      snprintf(line, sizeof(line), "mag: matrix diag=[%.3f %.3f %.3f]", g_mag_bias.soft_iron_matrix[0][0],
               g_mag_bias.soft_iron_matrix[1][1], g_mag_bias.soft_iron_matrix[2][2]);
      publish_status(event->client, topic_status, hostname, line);

      gyro_calibration_t gyro_cal = {0};
      esp_err_t gyro_ret = load_gyro_calibration_from_nvs(&gyro_cal);
      if (gyro_ret == ESP_OK && gyro_cal.version == 1) {
        snprintf(line, sizeof(line), "gyro: bias=[%.3f %.3f %.3f] ver=%u (NVS)", gyro_cal.gyro_bias[0],
                 gyro_cal.gyro_bias[1], gyro_cal.gyro_bias[2], (unsigned)gyro_cal.version);
      } else {
        snprintf(line, sizeof(line), "gyro: bias=[%.3f %.3f %.3f] (runtime fallback, NVS=%s)", g_gyro_bias.gyro_x,
                 g_gyro_bias.gyro_y, g_gyro_bias.gyro_z, esp_err_to_name(gyro_ret));
      }
      publish_status(event->client, topic_status, hostname, line);
      break;
    }

    // config show - show current motor configuration
    if (strcasecmp(msg, "config show") == 0) {
      char config_info[200];
      snprintf(config_info, sizeof(config_info),
               "config: wheel_calibration=%.2f,%.2f drive_kick=%u,%ums turn_kick=%u,%ums default_speed=%u kick=%s",
               g_motor_config.left_motor_speed_multiplier, g_motor_config.right_motor_speed_multiplier,
               g_motor_config.kick_speed, g_motor_config.kick_duration_ms,
               g_motor_config.kick_speed, // Using same for turn for now
               g_motor_config.kick_duration_ms, g_motor_config.default_drive_speed,
               g_motor_config.use_kick_start ? "enabled" : "disabled");
      publish_status(event->client, topic_status, hostname, config_info);
      break;
    }

    // config set drive_kick <speed> <duration> - configure drive kick-start parameters
    if (strncasecmp(msg, "config set drive_kick ", 22) == 0) {
      const char *p = msg + 22;
      unsigned speed = 0, duration = 0;
      if (sscanf(p, "%u %u", &speed, &duration) == 2 && speed >= 16 && speed <= 63 && duration >= 50 &&
          duration <= 500) {
        g_motor_config.kick_speed = (uint8_t)speed;
        g_motor_config.kick_duration_ms = (uint16_t)duration;
        g_motor_config.use_kick_start = true;
        char info[80];
        snprintf(info, sizeof(info), "drive kick updated: speed=%u duration=%ums", speed, duration);
        publish_status(event->client, topic_status, hostname, info);
        break;
      }
    }

    // config set turn_kick <speed> <duration> - configure turn kick-start parameters
    if (strncasecmp(msg, "config set turn_kick ", 21) == 0) {
      const char *p = msg + 21;
      unsigned speed = 0, duration = 0;
      if (sscanf(p, "%u %u", &speed, &duration) == 2 && speed >= 10 && speed <= 63 && duration >= 25 &&
          duration <= 500) {
        // For now, both drive and turn use the same kick parameters
        // In future, we could add separate turn_kick_speed and turn_kick_duration fields
        g_motor_config.kick_speed = (uint8_t)speed;
        g_motor_config.kick_duration_ms = (uint16_t)duration;
        g_motor_config.use_kick_start = true;
        char info[80];
        snprintf(info, sizeof(info), "turn kick updated: speed=%u duration=%ums", speed, duration);
        publish_status(event->client, topic_status, hostname, info);
        break;
      }
    }

    // config set wheel_calibration <left> <right> - configure wheel speed calibration
    if (strncasecmp(msg, "config set wheel_calibration ", 28) == 0) {
      const char *p = msg + 28;
      float left = 0.0f, right = 0.0f;
      if (sscanf(p, "%f %f", &left, &right) == 2 && left >= 0.5f && left <= 2.0f && right >= 0.5f && right <= 2.0f) {
        g_motor_config.left_motor_speed_multiplier = left;
        g_motor_config.right_motor_speed_multiplier = right;
        char info[80];
        snprintf(info, sizeof(info), "wheel calibration updated: left=%.2f right=%.2f", left, right);
        publish_status(event->client, topic_status, hostname, info);
        break;
      }
    }

    // config set default_speed <speed> - configure default drive speed
    if (strncasecmp(msg, "config set default_speed ", 25) == 0) {
      const char *p = msg + 25;
      unsigned speed = 0;
      if (sscanf(p, "%u", &speed) == 1 && speed <= 63) {
        g_motor_config.default_drive_speed = (uint8_t)speed;
        char info[80];
        snprintf(info, sizeof(info), "default drive speed updated: 0x%02X (%u)", g_motor_config.default_drive_speed,
                 speed);
        publish_status(event->client, topic_status, hostname, info);
        break;
      }
    }

    // sensor_loop <start|stop|counted> [iterations] [delay_ms]
    const char *prefix_mode = "sensor_loop";
    if (strncasecmp(msg, prefix_mode, strlen(prefix_mode)) == 0) {
      const char *p = msg + strlen(prefix_mode);
      while (*p && isspace((unsigned char)*p))
        p++;

	      char mode[16] = {0};
	      unsigned iterations = 0, delay_ms = 0;
	      int parsed = sscanf(p, "%15s %u %u", mode, &iterations, &delay_ms);

	      if (parsed < 1) {
	        ESP_LOGW(TAG, "Invalid sensor_loop command: '%s'", msg);
	        publish_status(event->client, topic_status, hostname,
	                                "Invalid sensor_loop command. Usage: sensor_loop <start|stop|counted> [iterations] [delay_ms]");
	        break;
	      }

			      if (strcasecmp(mode, "stop") == 0) {
			        g_sensor_loop_mode = SENSOR_LOOP_STOP;
			        g_sensor_loop_iterations = 0;
			      } else if (strcasecmp(mode, "start") == 0) {
			        g_sensor_loop_mode = SENSOR_LOOP_INFINITE;
			        g_sensor_loop_iterations = 0;
		        if (parsed >= 2)
		          g_sensor_loop_delay_ms = iterations; // treat 2nd arg as delay
		      } else if (strcasecmp(mode, "counted") == 0) {
	        g_sensor_loop_mode = SENSOR_LOOP_COUNTED;
	        g_sensor_loop_iterations = iterations;
		        if (parsed >= 3)
		          g_sensor_loop_delay_ms = delay_ms;
		      } else {
		        char err[128];
		        snprintf(err, sizeof(err),
		                 "Invalid sensor_loop mode '%s'. Usage: sensor_loop <start|stop|counted> [iterations] [delay_ms]",
		                 mode);
		        ESP_LOGW(TAG, "%s", err);
		        publish_status(event->client, topic_status, hostname,
		                                err);
		        break;
		      }

	      const char *loop_mode_name = (g_sensor_loop_mode == SENSOR_LOOP_STOP)     ? "stop"
	                                   : (g_sensor_loop_mode == SENSOR_LOOP_INFINITE) ? "infinite"
	                                                                                : "counted";
      char loop_info[128];
      if (g_sensor_loop_mode == SENSOR_LOOP_COUNTED) {
        snprintf(loop_info, sizeof(loop_info), "%s Sensor loop mode=%s, count=%u delay=%u ms", hostname, loop_mode_name,
                 (unsigned)g_sensor_loop_iterations, (unsigned)g_sensor_loop_delay_ms);
      } else if (g_sensor_loop_mode == SENSOR_LOOP_INFINITE) {
        snprintf(loop_info, sizeof(loop_info), "%s Sensor loop mode=%s, delay=%u ms", hostname, loop_mode_name,
                 (unsigned)g_sensor_loop_delay_ms);
      } else {
        snprintf(loop_info, sizeof(loop_info), "%s Sensor loop mode=%s", hostname, loop_mode_name);
      }
	      ESP_LOGI(TAG, "%s", loop_info);
      publish_status(event->client, topic_status, hostname, loop_info);

      if (g_sensor_loop_mode != SENSOR_LOOP_STOP) {
        char fusion_info[96];
        snprintf(fusion_info, sizeof(fusion_info), "%s Sensor fusion mode=%s", hostname,
                 sensor_fusion_mode_name(g_sensor_stream_mode));
        ESP_LOGI(TAG, "%s", fusion_info);
        publish_status(event->client, topic_status, hostname, fusion_info);
      }
      break;
    }

    if (strcasecmp(msg, "show sensor_fusion_mode") == 0) {
      char info[64];
      snprintf(info, sizeof(info), "Sensor fusion mode=%s", sensor_fusion_mode_name(g_sensor_stream_mode));
      ESP_LOGI(TAG, "%s", info);
      publish_status(event->client, topic_status, hostname, info);
      break;
    }

    if (strncasecmp(msg, "sensor_fusion_mode", 18) == 0) {
      const char *p = msg + 18;
      while (*p && isspace((unsigned char)*p))
        p++;

      char submode[16] = {0};
      int parsed_mode = sscanf(p, "%15s", submode);
      if (parsed_mode < 1) {
        publish_status(event->client, topic_status, hostname,
                                "Usage: sensor_fusion_mode <mag_accel_gyro|accel_gyro|mag|mag_accel>");
        break;
      }

      if (strcasecmp(submode, "accel_gyro") == 0) {
        g_sensor_stream_mode = MODE_ACCEL_GYRO;
      } else if (strcasecmp(submode, "mag_accel_gyro") == 0) {
        g_sensor_stream_mode = MODE_MAG_ACCEL_GYRO;
      } else if (strcasecmp(submode, "mag") == 0) {
        g_sensor_stream_mode = MODE_MAG;
      } else if (strcasecmp(submode, "mag_accel") == 0) {
        g_sensor_stream_mode = MODE_MAG_ACCEL;
      } else {
        publish_status(event->client, topic_status, hostname,
                                "Usage: sensor_fusion_mode <mag_accel_gyro|accel_gyro|mag|mag_accel>");
        break;
      }

      char info[80];
      snprintf(info, sizeof(info), "Sensor fusion mode=%s", sensor_fusion_mode_name(g_sensor_stream_mode));
      ESP_LOGI(TAG, "%s", info);
      publish_status(event->client, topic_status, hostname, info);
      break;
    }

    // sensor_stream <start|stop> [period_ms]
    if (strncasecmp(msg, "sensor_stream", 13) == 0) {
      const char *p = msg + 13;
      while (*p && isspace((unsigned char)*p))
        p++;
      char mode[16];
      unsigned period_ms = 0;
      int parsed = sscanf(p, "%15s %u", mode, &period_ms);

      if (strcasecmp(mode, "stop") == 0) {
        g_sensor_stream_enabled = false;
      } else if (strcasecmp(mode, "start") == 0) {
        g_sensor_stream_enabled = true;
        if (parsed >= 2 && period_ms >= 20) {
          g_sensor_stream_period_ms = period_ms;
        }
      } else {
        publish_status(event->client, topic_status, hostname, "Usage: sensor_stream <start|stop> [period_ms]");
        break;
      }

      char info[96];
      snprintf(info, sizeof(info), "Sensor stream %s (period=%ums)", g_sensor_stream_enabled ? "on" : "off",
               (unsigned)g_sensor_stream_period_ms);
      publish_status(event->client, topic_status, hostname, info);
      break;
    }

    if (strncasecmp(msg, "calibrate ", 10) == 0) {
      char sensor_buf[16] = {0};
      char mode_buf[16] = {0};

      // Parse up to two tokens after "calibrate"
      int parsed = sscanf(msg + 10, "%15s %15s", sensor_buf, mode_buf);

      const char *sensor = sensor_buf;
      const char *mode = (parsed == 2) ? mode_buf : NULL;

      // --- Gyroscope Calibration ---
      if (strcasecmp(sensor, "gyro") == 0) {
        ESP_LOGI(TAG, "MQTT command: calibrate gyro -> recalibrate gyroscope");

        if (!g_icm20948_dev_handle) {
          publish_status(event->client, topic_status, hostname, "Gyro calibration failed: IMU handle not initialized");
          break;
        }

        icm20948_calibrate_gyro_bias(g_icm20948_dev_handle, GYRO_NUM_CALIBRATION_SAMPLES);

        gyro_calibration_t gyro_cal = {.version = 1};
        gyro_cal.gyro_bias[0] = g_gyro_bias.gyro_x;
        gyro_cal.gyro_bias[1] = g_gyro_bias.gyro_y;
        gyro_cal.gyro_bias[2] = g_gyro_bias.gyro_z;
        store_gyro_calibration_to_nvs(&gyro_cal);

        publish_status(event->client, topic_status, hostname, "Gyroscope recalibrated and saved to NVS");
        break;
      }

      // --- Magnetometer Calibration ---
      else if (strcasecmp(sensor, "mag") == 0) {
        if (!ensure_mag_available(event, topic_status, hostname, "Mag calibration")) {
          break;
        }

        if (mode && strcasecmp(mode, "capture") == 0) {
          unsigned requested_samples = MAG_CAPTURE_DEFAULT_SAMPLES;
          unsigned delay_ms = MAG_CAPTURE_DEFAULT_DELAY_MS;
          (void)sscanf(msg + 10, "%*s %*s %u %u", &requested_samples, &delay_ms);

          if (requested_samples < MAG_CAPTURE_MIN_SAMPLES) {
            requested_samples = MAG_CAPTURE_MIN_SAMPLES;
          } else if (requested_samples > MAG_CAPTURE_MAX_SAMPLES) {
            requested_samples = MAG_CAPTURE_MAX_SAMPLES;
          }
          if (delay_ms < 10) {
            delay_ms = 10;
          } else if (delay_ms > 300) {
            delay_ms = 300;
          }

          char start_msg[160];
          snprintf(start_msg, sizeof(start_msg),
                   "Starting 3D mag capture: samples=%u delay=%ums. Move in slow figure-8 with tilt.",
                   requested_samples, delay_ms);
          publish_status(event->client, topic_status, hostname, start_msg);
          ESP_LOGI(TAG, "%s", start_msg);

          const int max_attempts = (int)requested_samples * 20;
          const int64_t max_capture_time_us = 180LL * 1000000LL;
          const int64_t capture_start_us = esp_timer_get_time();
          int valid_samples = 0;
          int attempts = 0;
          int read_errors = 0;

          while (valid_samples < (int)requested_samples) {
            attempts++;
            int64_t elapsed_us = esp_timer_get_time() - capture_start_us;
            if (attempts > max_attempts || elapsed_us > max_capture_time_us) {
              char fail_msg[160];
              snprintf(fail_msg, sizeof(fail_msg), "Mag capture timeout: valid=%d/%u attempts=%d read_errors=%d",
                       valid_samples, requested_samples, attempts, read_errors);
              publish_status(event->client, topic_status, hostname, fail_msg);
              ESP_LOGW(TAG, "%s", fail_msg);
              break;
            }

            icm20948_mag_data_t mag_raw;
            esp_err_t read_ret = icm20948_read_mag_data(g_icm20948_dev_handle, &mag_raw);
            if (read_ret != ESP_OK) {
              read_errors++;
              if (read_errors <= 3 || (read_errors % 10) == 0) {
                char warn_msg[128];
                snprintf(warn_msg, sizeof(warn_msg), "Mag capture read error #%d: %s", read_errors,
                         esp_err_to_name(read_ret));
                publish_status(event->client, topic_status, hostname, warn_msg);
                ESP_LOGW(TAG, "%s", warn_msg);
              }
              vTaskDelay(pdMS_TO_TICKS(delay_ms));
              continue;
            }

            valid_samples++;
            char sample_msg[128];
            snprintf(sample_msg, sizeof(sample_msg), "capture sample %d/%u: mx=%.3f, my=%.3f, mz=%.3f", valid_samples,
                     requested_samples, mag_raw.mag_x, mag_raw.mag_y, mag_raw.mag_z);
            publish_status(event->client, topic_status, hostname, sample_msg);

            if ((valid_samples % 50) == 0 || valid_samples == (int)requested_samples) {
              char progress_msg[128];
              snprintf(progress_msg, sizeof(progress_msg), "Mag capture progress: %d/%u (errors=%d)", valid_samples,
                       requested_samples, read_errors);
              publish_status(event->client, topic_status, hostname, progress_msg);
              ESP_LOGI(TAG, "%s", progress_msg);
            }

            vTaskDelay(pdMS_TO_TICKS(delay_ms));
          }

          if (valid_samples == (int)requested_samples) {
            char done_msg[160];
            snprintf(done_msg, sizeof(done_msg),
                     "Mag capture complete: %d samples collected (errors=%d). Run host fit, then apply.", valid_samples,
                     read_errors);
            publish_status(event->client, topic_status, hostname, done_msg);
            ESP_LOGI(TAG, "%s", done_msg);
          }
          break;
        }

        if (mode && strcasecmp(mode, "apply") == 0) {
          mag_calibration_t mag_bias = {0};
          unsigned version = 3; // default for external 3D fit
          int parsed = sscanf(msg + 10,
                              "%*s %*s "
                              "%f %f %f "
                              "%f %f %f "
                              "%f %f %f "
                              "%f %f %f %u",
                              &mag_bias.hard_iron_bias[0], &mag_bias.hard_iron_bias[1], &mag_bias.hard_iron_bias[2],
                              &mag_bias.soft_iron_matrix[0][0], &mag_bias.soft_iron_matrix[0][1],
                              &mag_bias.soft_iron_matrix[0][2], &mag_bias.soft_iron_matrix[1][0],
                              &mag_bias.soft_iron_matrix[1][1], &mag_bias.soft_iron_matrix[1][2],
                              &mag_bias.soft_iron_matrix[2][0], &mag_bias.soft_iron_matrix[2][1],
                              &mag_bias.soft_iron_matrix[2][2], &version);

          if (parsed < 12) {
            esp_mqtt_client_publish(
                event->client, topic_status,
                "Apply failed: syntax -> calibrate mag apply bx by bz m00 m01 m02 m10 m11 m12 m20 m21 m22 [version]", 0,
                0, 0);
            break;
          }
          if (!(version == 1 || version == 3)) {
            publish_status(event->client, topic_status, hostname, "Apply failed: version must be 1 or 3");
            break;
          }

          bool finite_ok = true;
          for (int i = 0; i < 3; i++) {
            if (!isfinite(mag_bias.hard_iron_bias[i])) {
              finite_ok = false;
            }
            for (int j = 0; j < 3; j++) {
              if (!isfinite(mag_bias.soft_iron_matrix[i][j])) {
                finite_ok = false;
              }
            }
          }
          if (!finite_ok) {
            publish_status(event->client, topic_status, hostname, "Apply failed: non-finite bias/matrix value");
            break;
          }

          mag_bias.version = (uint8_t)version;
          esp_err_t nvs_ret = store_mag_calibration_to_nvs(&mag_bias);
          if (nvs_ret != ESP_OK) {
            char nvs_msg[128];
            snprintf(nvs_msg, sizeof(nvs_msg), "Apply failed: NVS save error (%s)", esp_err_to_name(nvs_ret));
            publish_status(event->client, topic_status, hostname, nvs_msg);
            break;
          }

          g_mag_bias = mag_bias;
          char ok_msg[160];
          snprintf(ok_msg, sizeof(ok_msg),
                   "Applied mag calibration: bias=[%.3f %.3f %.3f] diag=[%.3f %.3f %.3f] ver=%u",
                   mag_bias.hard_iron_bias[0], mag_bias.hard_iron_bias[1], mag_bias.hard_iron_bias[2],
                   mag_bias.soft_iron_matrix[0][0], mag_bias.soft_iron_matrix[1][1], mag_bias.soft_iron_matrix[2][2],
                   (unsigned)mag_bias.version);
          publish_status(event->client, topic_status, hostname, ok_msg);
          ESP_LOGI(TAG, "%s", ok_msg);
          break;
        }

        // Allocate calibration buffer on the heap
        raw_mag_sample_t *samples = malloc(sizeof(raw_mag_sample_t) * MAG_NUM_CALIBRATION_SAMPLES);
        if (!samples) {
          publish_status(event->client, topic_status, hostname, "Mag calibration failed: out of memory");
          ESP_LOGE(TAG, "Failed to allocate %d samples on heap", MAG_NUM_CALIBRATION_SAMPLES);
          break;
        }
        // raw_mag_sample_t samples[MAG_NUM_CALIBRATION_SAMPLES];
        mag_calibration_t mag_bias;
        esp_err_t status = ESP_FAIL;
        const char *failure_reason = "Magnetometer calibration failed";

        if (mode && strcasecmp(mode, "hard") == 0) {
          ESP_LOGI(TAG, "Starting 2D magnetometer calibration (flat rotation)...");
          publish_status(event->client, topic_status, hostname,
                                  "Starting 2D magnetometer calibration (flat rotation)...");

          // Rotate robot for 18 sec
          start_turn(TURN_RIGHT, 15, 18000);

          status = collect_mag_calibration_data(g_icm20948_dev_handle, samples);
          if (status != ESP_OK) {
            failure_reason = "Mag calibration failed: sensor read/collection timeout";
          }
          if (status == ESP_OK) {
            status = icm20948_calibrate_mag_2d(g_icm20948_dev_handle, samples, MAG_NUM_CALIBRATION_SAMPLES, &mag_bias);
            if (status != ESP_OK) {
              failure_reason = "Mag calibration failed: 2D solve failed";
            }
            mag_bias.version = 1;
          }
        } else if (mode && strcasecmp(mode, "soft") == 0) {
          ESP_LOGI(TAG, "\nMagnetometer 3D soft-iron calibration will start in 3 seconds.\n"
                        "Platform must be rotated slowly in a figure-8 pattern, covering all orientations!");
          publish_status(event->client, topic_status, hostname,
                                  "Magnetometer 3D soft-iron calibration (figure-8) will start in 3 sec...");
          vTaskDelay(pdMS_TO_TICKS(3000));

          status = collect_mag_calibration_data(g_icm20948_dev_handle, samples);
          if (status != ESP_OK) {
            failure_reason = "Mag calibration failed: sensor read/collection timeout";
          }
          if (status == ESP_OK) {
            status = icm20948_calibrate_mag_3d(g_icm20948_dev_handle, samples, MAG_NUM_CALIBRATION_SAMPLES, &mag_bias);
            if (status != ESP_OK) {
              failure_reason = "Mag calibration failed: 3D solve failed";
            }
            mag_bias.version = 3;
          }
        } else {
          publish_status(event->client, topic_status, hostname,
                                  "Unknown calibration mode (use hard|soft|capture [samples] [delay_ms]|apply ...)");
          free(samples);
          break;
        }

        if (status == ESP_OK) {
          esp_err_t nvs_ret = store_mag_calibration_to_nvs(&mag_bias);
          if (nvs_ret == ESP_OK) {
            g_mag_bias = mag_bias;
            publish_status(event->client, topic_status, hostname, "Magnetometer recalibrated and saved to NVS");
          } else {
            char nvs_msg[128];
            snprintf(nvs_msg, sizeof(nvs_msg), "Mag calibration solved, but NVS save failed: %s",
                     esp_err_to_name(nvs_ret));
            publish_status(event->client, topic_status, hostname, nvs_msg);
            status = nvs_ret;
            failure_reason = "Mag calibration failed: NVS save error";
          }

          ESP_LOGI(TAG, "Matrix diag: [%.3f, %.3f, %.3f]", mag_bias.soft_iron_matrix[0][0],
                   mag_bias.soft_iron_matrix[1][1], mag_bias.soft_iron_matrix[2][2]);

          char bias_msg[128];
          snprintf(bias_msg, sizeof(bias_msg), "Hard-Iron Bias: X=%.3f Y=%.3f Z=%.3f", mag_bias.hard_iron_bias[0],
                   mag_bias.hard_iron_bias[1], mag_bias.hard_iron_bias[2]);
          publish_status(event->client, topic_status, hostname, bias_msg);

          char matrix_msg[128];
          snprintf(matrix_msg, sizeof(matrix_msg), "Matrix diag: [%.3f, %.3f, %.3f]", mag_bias.soft_iron_matrix[0][0],
                   mag_bias.soft_iron_matrix[1][1], mag_bias.soft_iron_matrix[2][2]);
          publish_status(event->client, topic_status, hostname, matrix_msg);
        } else {
          char fail_msg[128];
          snprintf(fail_msg, sizeof(fail_msg), "%s (%s)", failure_reason, esp_err_to_name(status));
          publish_status(event->client, topic_status, hostname, fail_msg);
        }
        // Free heap buffer
        free(samples);
        break;
      }

      // --- Unknown Sensor ---
      else {
        char error_msg[80];
        snprintf(error_msg, sizeof(error_msg), "Unknown sensor type: %s (supported: gyro, mag)", sensor);
        publish_status(event->client, topic_status, hostname, error_msg);
        break;
      }
    }

    // rotate to <angle> [speed] [step_ms] [delay_ms] [tolerance]
    if (strncasecmp(msg, "rotate to ", 10) == 0) {
      if (!ensure_mag_available(event, topic_status, hostname, "Rotate to")) {
        break;
      }

      unsigned target_heading = 0;
      unsigned speed = 15, step_ms = 10, delay_ms = 300; // defaults
      float tolerance = 5.0f;                            // default stop window

      int parsed = sscanf(msg + 10, "%u %u %u %u %f", &target_heading, &speed, &step_ms, &delay_ms, &tolerance);

      if (parsed >= 1 && target_heading <= 360) {
        if (speed > 63)
          speed = 63;
        if (step_ms < 5)
          step_ms = 5;
        if (delay_ms < 100)
          delay_ms = 100;
        if (tolerance < 5.0f)
          tolerance = 5.0f;

        ESP_LOGI(TAG, "MQTT command: rotate to target=%u deg, speed=%u, step=%ums, delay=%ums, tol=%.2f",
                 target_heading, speed, step_ms, delay_ms, tolerance);

        char param_msg[128];
        snprintf(param_msg, sizeof(param_msg), "Rotate to params: target=%u speed=%u step=%ums delay=%ums tol=%.2f",
                 target_heading, speed, step_ms, delay_ms, tolerance);
        publish_status(event->client, topic_status, hostname, param_msg);

        sensor_loop_mode_t prev_mode = g_sensor_loop_mode;
        g_sensor_loop_mode = SENSOR_LOOP_INFINITE;

        int max_iters = 100;

        while (max_iters-- > 0) {
          // Read calibrated magnetometer
          icm20948_mag_data_t mag_cal;
          if (icm20948_read_mag_calibrated(g_icm20948_dev_handle, &g_mag_bias, &mag_cal) != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read calibrated magnetometer during rotate-to");
            break;
          }
          float heading = icm20948_compute_heading(&mag_cal);

          char heading_msg[80];
          snprintf(heading_msg, sizeof(heading_msg), "Current heading=%.2f target=%u", heading, target_heading);
          publish_status(event->client, topic_status, hostname, heading_msg);
          ESP_LOGI(TAG, "Heading step: %.2f deg (target %u)", heading, target_heading);

          float diff = target_heading - heading;
          while (diff > 180.0f)
            diff -= 360.0f;
          while (diff < -180.0f)
            diff += 360.0f;

          ESP_LOGI(TAG, "Heading diff=%.2f tolerance=%.2f", diff, tolerance);

          if (fabsf(diff) <= tolerance) {
            publish_status(event->client, topic_status, hostname, "Rotate to complete");
            break;
          }

          // --- Fixed speed (no proportional ramp) ---
          uint8_t turn_speed = speed;

          // Decide direction
          robot_turn_direction_t dir = (diff > 0) ? TURN_RIGHT : TURN_LEFT;

          // --- Serialized motor command ---
          esp_err_t motor_ret = turn_robot(g_left_motor_dev_handle, g_right_motor_dev_handle, dir, turn_speed,
                                           turn_speed, step_ms, &g_motor_config);
          if (motor_ret != ESP_OK) {
            ESP_LOGE(TAG, "Rotate-to: motor command failed: %s", esp_err_to_name(motor_ret));
          }

          // Pause before next iteration
          vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        if (max_iters <= 0) {
          publish_status(event->client, topic_status, hostname, "Rotate to timeout, not aligned");
        }

        g_sensor_loop_mode = prev_mode;
      }
      break;
    }

    if (strncasecmp(msg, "show raw accel", 14) == 0) {
      unsigned count = 1, delay_ms = 50; // defaults
      sscanf(msg + 14, "%u %u", &count, &delay_ms);
      if (count == 0)
        count = 1;
      if (delay_ms < 10)
        delay_ms = 10;

      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      unsigned valid_samples = 0;

      for (unsigned i = 0; i < count; i++) {
        icm20948_data_t sensor_raw;
        esp_err_t ret = icm20948_read_sensor_data(g_icm20948_dev_handle, &sensor_raw);

        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Show raw accel failed: IMU read error on sample %u", i + 1);
          continue;
        }

        sum_x += sensor_raw.accel_x;
        sum_y += sensor_raw.accel_y;
        sum_z += sensor_raw.accel_z;
        valid_samples++;

        char sample_msg[144];
        snprintf(sample_msg, sizeof(sample_msg), "Raw accel sample %u: x=%.3f y=%.3f z=%.3f g", i + 1,
                 sensor_raw.accel_x, sensor_raw.accel_y, sensor_raw.accel_z);
        publish_status(event->client, topic_status, hostname, sample_msg);
        ESP_LOGI(TAG, "%s", sample_msg);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (valid_samples > 0) {
        char msg_out[176];
        snprintf(msg_out, sizeof(msg_out),
                 "Average of %u raw accel reading(s) (delay=%ums): x=%.3f y=%.3f z=%.3f g",
                 valid_samples, delay_ms, sum_x / valid_samples, sum_y / valid_samples, sum_z / valid_samples);
        publish_status(event->client, topic_status, hostname, msg_out);
        ESP_LOGI(TAG, "%s", msg_out);
      } else {
        publish_status(event->client, topic_status, hostname, "Show raw accel failed: no valid samples");
        ESP_LOGW(TAG, "Show raw accel failed: no valid samples");
      }
      break;
    }

    if (strncasecmp(msg, "show calibrated gyro", 20) == 0) {
      unsigned count = 1, delay_ms = 50; // defaults
      sscanf(msg + 20, "%u %u", &count, &delay_ms);
      if (count == 0)
        count = 1;
      if (delay_ms < 10)
        delay_ms = 10;

      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      unsigned valid_samples = 0;

      for (unsigned i = 0; i < count; i++) {
        icm20948_data_t gyro_cal;
        esp_err_t ret = icm20948_read_gyro_calibrated(g_icm20948_dev_handle, &g_gyro_bias, &gyro_cal);

        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Show calibrated gyro failed: IMU read error on sample %u", i + 1);
          continue;
        }

        sum_x += gyro_cal.gyro_x;
        sum_y += gyro_cal.gyro_y;
        sum_z += gyro_cal.gyro_z;
        valid_samples++;

        char sample_msg[160];
        snprintf(sample_msg, sizeof(sample_msg), "Calibrated gyro sample %u: x=%.3f y=%.3f z=%.3f dps", i + 1,
                 gyro_cal.gyro_x, gyro_cal.gyro_y, gyro_cal.gyro_z);
        publish_status(event->client, topic_status, hostname, sample_msg);
        ESP_LOGI(TAG, "%s", sample_msg);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (valid_samples > 0) {
        char msg_out[192];
        snprintf(msg_out, sizeof(msg_out),
                 "Average of %u calibrated gyro reading(s) (delay=%ums): x=%.3f y=%.3f z=%.3f dps",
                 valid_samples, delay_ms, sum_x / valid_samples, sum_y / valid_samples, sum_z / valid_samples);
        publish_status(event->client, topic_status, hostname, msg_out);
        ESP_LOGI(TAG, "%s", msg_out);
      } else {
        publish_status(event->client, topic_status, hostname, "Show calibrated gyro failed: no valid samples");
        ESP_LOGW(TAG, "Show calibrated gyro failed: no valid samples");
      }
      break;
    }

    if (strncasecmp(msg, "show raw mag", 12) == 0) {
      if (!ensure_mag_available(event, topic_status, hostname, "Show raw mag")) {
        break;
      }

      unsigned count = 1, delay_ms = 50; // defaults
      sscanf(msg + 12, "%u %u", &count, &delay_ms);
      if (count == 0)
        count = 1;
      if (delay_ms < 10)
        delay_ms = 10;

      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      unsigned valid_samples = 0;

      for (unsigned i = 0; i < count; i++) {
        icm20948_mag_data_t mag_raw;
        esp_err_t ret = icm20948_read_mag_data(g_icm20948_dev_handle, &mag_raw);

        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Show raw mag failed: IMU read error on sample %u", i + 1);
          continue;
        }

        sum_x += mag_raw.mag_x;
        sum_y += mag_raw.mag_y;
        sum_z += mag_raw.mag_z;
        valid_samples++;

        char sample_msg[128];
        snprintf(sample_msg, sizeof(sample_msg), "Raw mag sample %u: x=%.2f y=%.2f z=%.2f uT", i + 1,
                 mag_raw.mag_x, mag_raw.mag_y, mag_raw.mag_z);
        publish_status(event->client, topic_status, hostname, sample_msg);
        ESP_LOGI(TAG, "%s", sample_msg);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (valid_samples > 0) {
        char msg_out[160];
        snprintf(msg_out, sizeof(msg_out),
                 "Average of %u raw mag reading(s) (delay=%ums): x=%.2f y=%.2f z=%.2f uT", valid_samples,
                 delay_ms, sum_x / valid_samples, sum_y / valid_samples, sum_z / valid_samples);
        publish_status(event->client, topic_status, hostname, msg_out);
        ESP_LOGI(TAG, "%s", msg_out);
      } else {
        publish_status(event->client, topic_status, hostname, "Show raw mag failed: no valid samples");
        ESP_LOGW(TAG, "Show raw mag failed: no valid samples");
      }
      break;
    }

    if (strncasecmp(msg, "show calibrated mag", 19) == 0) {
      if (!ensure_mag_available(event, topic_status, hostname, "Show calibrated mag")) {
        break;
      }

      unsigned count = 1, delay_ms = 50; // defaults
      sscanf(msg + 19, "%u %u", &count, &delay_ms);
      if (count == 0)
        count = 1;
      if (delay_ms < 10)
        delay_ms = 10;

      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      unsigned valid_samples = 0;

      for (unsigned i = 0; i < count; i++) {
        icm20948_mag_data_t mag_cal;
        esp_err_t ret = icm20948_read_mag_calibrated(g_icm20948_dev_handle, &g_mag_bias, &mag_cal);

        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Show calibrated mag failed: IMU read error on sample %u", i + 1);
          continue;
        }

        sum_x += mag_cal.mag_x;
        sum_y += mag_cal.mag_y;
        sum_z += mag_cal.mag_z;
        valid_samples++;

        char sample_msg[144];
        snprintf(sample_msg, sizeof(sample_msg), "Calibrated mag sample %u: x=%.2f y=%.2f z=%.2f uT", i + 1,
                 mag_cal.mag_x, mag_cal.mag_y, mag_cal.mag_z);
        publish_status(event->client, topic_status, hostname, sample_msg);
        ESP_LOGI(TAG, "%s", sample_msg);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (valid_samples > 0) {
        char msg_out[176];
        snprintf(msg_out, sizeof(msg_out),
                 "Average of %u calibrated mag reading(s) (delay=%ums): x=%.2f y=%.2f z=%.2f uT",
                 valid_samples, delay_ms, sum_x / valid_samples, sum_y / valid_samples, sum_z / valid_samples);
        publish_status(event->client, topic_status, hostname, msg_out);
        ESP_LOGI(TAG, "%s", msg_out);
      } else {
        publish_status(event->client, topic_status, hostname, "Show calibrated mag failed: no valid samples");
        ESP_LOGW(TAG, "Show calibrated mag failed: no valid samples");
      }
      break;
    }

    if (strncasecmp(msg, "show heading", 12) == 0) {
      if (!ensure_mag_available(event, topic_status, hostname, "Show heading")) {
        break;
      }

      unsigned count = 1, delay_ms = 50; // defaults
      sscanf(msg + 12, "%u %u", &count, &delay_ms);
      if (count == 0)
        count = 1;
      if (delay_ms < 10)
        delay_ms = 10; // enforce minimum delay

      float sum_x = 0.0f, sum_y = 0.0f; // vector sums for heading
      float sum_mag = 0.0f, sum_mag_sq = 0.0f;
      unsigned valid_samples = 0;

      for (unsigned i = 0; i < count; i++) {
        icm20948_mag_data_t mag_cal;
        esp_err_t ret = icm20948_read_mag_calibrated(g_icm20948_dev_handle, &g_mag_bias, &mag_cal);

        if (ret != ESP_OK) {
          ESP_LOGW(TAG, "Show heading failed: IMU read error on sample %u", i + 1);
          continue; // skip invalid reads
        }

        float heading = icm20948_compute_heading(&mag_cal);
        float rad = heading * M_PI / 180.0f;
        float x = cosf(rad);
        float y = sinf(rad);

        sum_x += x;
        sum_y += y;

        float mag_norm =
            sqrtf(mag_cal.mag_x * mag_cal.mag_x + mag_cal.mag_y * mag_cal.mag_y + mag_cal.mag_z * mag_cal.mag_z);

        sum_mag += mag_norm;
        sum_mag_sq += mag_norm * mag_norm;
        valid_samples++;

        ESP_LOGI(TAG, "Sample %u: heading=%.2f deg, magnitude=%.2f uT", i + 1, heading, mag_norm);

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
      }

      if (valid_samples > 0) {
        // Vector average for heading
        float avg_x = sum_x / valid_samples;
        float avg_y = sum_y / valid_samples;
        float avg_rad = atan2f(avg_y, avg_x);
        float avg_heading = avg_rad * 180.0f / M_PI;
        if (avg_heading < 0.0f)
          avg_heading += 360.0f;

        // Circular standard deviation
        float R = sqrtf(avg_x * avg_x + avg_y * avg_y);
        float circ_std = sqrtf(-2.0f * logf(R)) * 180.0f / M_PI;

        // Magnitude average and stddev
        float avg_mag = sum_mag / valid_samples;
        float var_mag = (sum_mag_sq / valid_samples) - (avg_mag * avg_mag);
        float std_mag = sqrtf(var_mag > 0 ? var_mag : 0);

        char msg_out[160];
        snprintf(msg_out, sizeof(msg_out),
                 "Average of %u reading(s) (delay=%ums): heading=%.2f +/- %.2f deg, magnitude=%.2f +/- %.2f uT",
                 valid_samples, delay_ms, avg_heading, circ_std, avg_mag, std_mag);

        publish_status(event->client, topic_status, hostname, msg_out);
        ESP_LOGI(TAG, "%s", msg_out);
      } else {
        publish_status(event->client, topic_status, hostname, "Show heading failed: no valid samples");
        ESP_LOGW(TAG, "Show heading failed: no valid samples");
      }
      break;
    }

    // turn to <angle> [speed] [divisor]
    if (strncasecmp(msg, "turn to ", 8) == 0) {
      if (!ensure_mag_available(event, topic_status, hostname, "Turn to")) {
        break;
      }

      unsigned target_heading = 0;
      unsigned speed = 20;   // default
      float divisor = 4.0f;  // default speed ramp divisor
      if (sscanf(msg + 8, "%u %u %f", &target_heading, &speed, &divisor) >= 1) {
        if (speed > 63)
          speed = 63;
        if (divisor < 2.0f)
          divisor = 2.0f;
        if (divisor > 4.0f)
          divisor = 4.0f;

        ESP_LOGI(TAG, "MQTT command: turn to target=%u deg, speed=%u, divisor=%.2f", target_heading, speed, divisor);

        const float tolerance = 5.0f; // stop window
        int max_iters = 100;          // safety timeout (~10s at 100ms/sample)

        while (max_iters-- > 0) {
          // --- Average multiple magnetometer samples ---
          const unsigned avg_samples = 3;
          float sum_x = 0.0f, sum_y = 0.0f;
          unsigned valid_samples = 0;

          for (unsigned j = 0; j < avg_samples; j++) {
            icm20948_mag_data_t mag_cal;
            if (icm20948_read_mag_calibrated(g_icm20948_dev_handle, &g_mag_bias, &mag_cal) == ESP_OK) {
              float heading = icm20948_compute_heading(&mag_cal);
              float rad = heading * M_PI / 180.0f;
              sum_x += cosf(rad);
              sum_y += sinf(rad);
              valid_samples++;
            }
            vTaskDelay(pdMS_TO_TICKS(20)); // small pause between samples
          }

          if (valid_samples == 0) {
            publish_status(event->client, topic_status, hostname, "Turn to failed: IMU read error");
            break;
          }

          // Vector average heading
          float avg_rad = atan2f(sum_y, sum_x);
          float avg_heading = avg_rad * 180.0f / M_PI;
          if (avg_heading < 0.0f)
            avg_heading += 360.0f;

          float diff = target_heading - avg_heading;
          while (diff > 180.0f)
            diff -= 360.0f;
          while (diff < -180.0f)
            diff += 360.0f;

          // Stop immediately if the current averaged heading is already inside tolerance.
          if (fabsf(diff) <= tolerance) {
            char heading_msg[100];
            char done_msg[120];
            snprintf(heading_msg, sizeof(heading_msg), "AvgHeading=%.2f target=%u diff=%.2f aligned", avg_heading,
                     target_heading, diff);
            publish_status(event->client, topic_status, hostname, heading_msg);
            stop_all_motors();
            snprintf(done_msg, sizeof(done_msg), "Turn to complete: heading=%.2f target=%u diff=%.2f", avg_heading,
                     target_heading, diff);
            publish_status(event->client, topic_status, hostname, done_msg);
            break;
          }

          // Proportional speed ramp
          uint8_t turn_speed = (uint8_t)(fabsf(diff) / divisor);
          if (turn_speed < 12)
            turn_speed = 12;
          if (turn_speed > speed)
            turn_speed = speed;

          // Decide direction
          robot_turn_direction_t dir = (diff > 0) ? TURN_RIGHT : TURN_LEFT;

          // Serialized motor command
          turn_robot(g_left_motor_dev_handle, g_right_motor_dev_handle, dir, turn_speed, turn_speed,
                     100, // short duration per step (ms)
                     &g_motor_config);

          // Publish averaged heading used for this turn step.
          char heading_msg[100];
          snprintf(heading_msg, sizeof(heading_msg), "AvgHeading=%.2f target=%u diff=%.2f speed=%u", avg_heading,
                   target_heading, diff, turn_speed);
          publish_status(event->client, topic_status, hostname, heading_msg);

          vTaskDelay(pdMS_TO_TICKS(100)); // pause before next iteration
        }

        if (max_iters <= 0) {
          stop_all_motors();
          publish_status(event->client, topic_status, hostname, "Turn to timeout, not aligned");
        }
        break;
      }
    }

    // If no command matched, optionally echo help
    ESP_LOGW(TAG, "Unknown command: '%s'", msg);
    publish_status(event->client, topic_status, hostname, "Unrecognized command");
    break;
  }
  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "MQTT error event");
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGW(TAG, "MQTT disconnected from broker");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT successfully subscribed to topic, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT successfully unsubscribed from topic, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(TAG, "MQTT message published successfully, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_BEFORE_CONNECT:
    ESP_LOGI(TAG, "MQTT attempting to connect to broker");
    break;
  case MQTT_EVENT_DELETED:
    ESP_LOGW(TAG, "MQTT client deleted");
    break;
  case MQTT_EVENT_ANY:
  case MQTT_USER_EVENT:
  default:
    ESP_LOGD(TAG, "MQTT other event received: %d", (int)event_id);
    break;
  }
} // mqtt_event_handler(

// Function to publish position to the broadcast channel
void publish_position(esp_mqtt_client_handle_t client, const char *hostname, int x, int y)
{
  char pos_msg[64];
  snprintf(pos_msg, sizeof(pos_msg), "%s position: x=%d, y=%d", hostname, x, y);
  esp_mqtt_client_publish(client, "arena/all/position", pos_msg, 0, 0, 0);
}
