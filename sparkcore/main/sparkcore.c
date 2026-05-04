/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparkcore.c - Main SynchroSpark core application
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
 
/*
 * Important:
 * Do not flash this program to the device, it will be downloaded wirelessly by the OTA updater.
 * This program assumes that sparknode_ota_updater is already placed in the factory partition of
 * the device.
 */

#define OTA_UPDATE_ENABLED

#include "bmp388.h"
#include "drv5033.h"
#include "drv8830.h"
#include "fusion_interface.h"
#include "esp32_chip_info.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "icm20948.h"
#include "mqtt_handler.h"
#include "network_config.h"
#include "ota_utils.h"
#include "rgb_led_ring.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "wifi_credentials.h"
#include "wifi_net.h"
#include <nvs_utils.h>
#include <stdio.h>

/*
Attention:

In our standard design, the RGB LED ring connects to GPIO 2 of the ESP32-CAM board. However,
GPIO 2 is one of the ESP32-CAM's strapping pins, meaning that if it is pulled high during boot,
the ESP32-CAM will not enter flashing mode. To address this issue, Ismet has designed a small
circuit on the camera back board that ensures GPIO 2 remains low during startup.

After flashing "sparknode_ota_updater" to the device via USB cable connection, we can safely
continue using GPIO 2 to control the RGB LED ring since subsequent firmware updates use the OTA
method. If USB flashing is needed again, either the back board circuit must be present to
hold GPIO 2 low during boot, or the RGB LED ring must be unplugged from GPIO 2 before flashing
the device.
*/

// Extern declarations for global sensor data (defined in syncspark_config.c)
// Inertial Measurement Unit (IMU) data structures and globals
extern icm20948_data_t g_sensor_data;  // Current gyro + accel data
extern icm20948_mag_data_t g_mag_data; // Current magnetometer data
extern volatile sensor_loop_mode_t g_sensor_loop_mode;
extern volatile uint32_t g_sensor_loop_iterations;
extern volatile uint32_t g_sensor_loop_delay_ms;
extern volatile bool g_sensor_stream_enabled;
extern volatile uint32_t g_sensor_stream_period_ms;
extern volatile sensor_stream_mode_t g_sensor_stream_mode;

static const char *TAG = "sparkcore";

static const float FUSION_GAIN_DEFAULT = 0.5f;
static const float FUSION_GAIN_ACCEL_GYRO = 3.0f;
static const float FUSION_GAIN_MAG_ACCEL = 3.0f;
static const float FUSION_GAIN_MAG_ACCEL_GYRO = 1.5f;

void initialize_i2c_master(i2c_master_bus_handle_t *p_i2c_bus_handle)
{
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, p_i2c_bus_handle));
  ESP_LOGI("I2C", "I2C master initialization is successful.");
} // initialize_i2c_master()

void i2c_scan(i2c_master_bus_handle_t i2c_bus_handle)
{
  int i;
  int devices_found = 0;
  esp_err_t espRc;

  ESP_LOGI(TAG, "Starting I2C scan...");
  for (i = 1; i < 127; i++) {
    espRc = i2c_master_probe(i2c_bus_handle, i, -1);
    if (espRc == ESP_OK) {
      ESP_LOGI(TAG, "Found device at 0x%02x.", i);
      devices_found++;
    }
  }

  if (devices_found == 0) {
    ESP_LOGW(TAG, "No I2C devices detected!");
  } else {
    ESP_LOGI(TAG, "I2C scan complete, %d device(s) found.\n", devices_found);
  }

} // i2c_scan()

void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) { // Blink the red LED continuously
    blink_esp32cam_red_led(200, 1000);
  }
} // task_blink_esp32cam_red_led()

void task_udp_heartbeat(void *pvParameter)
{
  static uint32_t i = 1;
  while (1) {
    blink_esp32cam_flash_led(9, 100);
    blink_esp32cam_flash_led(9, 100);

    // Get WiFi RSSI
    int rssi = -999;
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
      rssi = ap_info.rssi;
    }
    ESP_LOGI("task UDP heartbeat", "(id %d) #%" PRIu32 " RSSI=%d dBm", g_sparknode_id, i++, rssi);
    vTaskDelay(pdMS_TO_TICKS(30000)); // Every 30 seconds
  }
} // task_udp_heartbeat()

void task_debug_hall_sensors(void *pvParameter)
{
  char *TAG = "hall_debug";
  uint32_t last_left_count = 0;
  uint32_t last_right_count = 0;

  while (1) {
    // Get current sensor states and counts
    bool sensor_1_state, sensor_2_state;
    get_sensor_states(&sensor_1_state, &sensor_2_state);
    uint32_t left_count = get_sensor_count(SENSOR_1);  // Left wheel
    uint32_t right_count = get_sensor_count(SENSOR_2); // Right wheel

    // Log when individual wheel counts change
    if (left_count != last_left_count) {
      float revolutions = calculate_wheel_revolutions(SENSOR_1);
      float angle = calculate_wheel_angle_degrees(SENSOR_1);
      ESP_LOGI(TAG, "L. wheel count - %lu -> %lu (+%lu) | Revs: %.3f | Angle: %.1f°", (unsigned long)last_left_count,
               (unsigned long)left_count, (unsigned long)(left_count - last_left_count), revolutions, angle);
      last_left_count = left_count;
    }

    if (right_count != last_right_count) {
      float revolutions = calculate_wheel_revolutions(SENSOR_2);
      float angle = calculate_wheel_angle_degrees(SENSOR_2);
      ESP_LOGI(TAG, "R. wheel count - %lu -> %lu (+%lu) | Revs: %.3f | Angle: %.1f°", (unsigned long)last_right_count,
               (unsigned long)right_count, (unsigned long)(right_count - last_right_count), revolutions, angle);
      last_right_count = right_count;
    }

    // Check if wheels are moving and calculate RPM for each
    bool left_moving = is_wheel_moving(SENSOR_1, 1000);
    bool right_moving = is_wheel_moving(SENSOR_2, 1000);

    if (left_moving) {
      float rpm = calculate_wheel_rpm(SENSOR_1, 2000);
      if (rpm > 0) {
        // ESP_LOGI(TAG, "Left wheel moving - RPM = %.1f", rpm);
      }
    }

    if (right_moving) {
      float rpm = calculate_wheel_rpm(SENSOR_2, 2000);
      if (rpm > 0) {
        // ESP_LOGI(TAG, "Right wheel moving - RPM = %.1f", rpm);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(250)); // Check periodically
  }
} // task_debug_hall_sensors()

// Structure to pass parameters to sensor fusion task
typedef struct {
  i2c_master_dev_handle_t icm20948_dev_handle;
  mag_calibration_t mag_bias;
  icm20948_data_t gyro_bias;
} sensor_fusion_params_t;

void task_sensor_fusion(void *pvParameter)
{
  sensor_fusion_params_t *params = (sensor_fusion_params_t *)pvParameter;
  char *TAG = "task_sensor_fusion";
  static int sample_count = 1;

  ESP_LOGI(TAG, "Sensor fusion task started");

  // Timing state for adaptive dt
  static uint64_t last_us = 0;
  static uint64_t last_stream_us = 0;
  static int accel_gyro_seed_pending = 0;
  static int accel_gyro_seed_samples = 0;
  static float accel_gyro_seed_sum_sin = 0.0f;
  static float accel_gyro_seed_sum_cos = 0.0f;
  static float accel_gyro_seed_heading = 0.0f;
  const int accel_gyro_seed_target = 5;

  while (1) {
    if (g_sensor_loop_mode == SENSOR_LOOP_STOP) {
      vTaskDelay(pdMS_TO_TICKS(100)); // idle
      continue;
    }
    if (g_sensor_loop_mode == SENSOR_LOOP_COUNTED && g_sensor_loop_iterations == 0) {
      g_sensor_loop_mode = SENSOR_LOOP_STOP;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Read sensors into global structures
    // Read calibrated gyro + accel
    icm20948_data_t calibrated_gyro_accel;
    esp_err_t gyro_ret =
        icm20948_read_gyro_calibrated(params->icm20948_dev_handle, &params->gyro_bias, &calibrated_gyro_accel);
    if (gyro_ret == ESP_OK) {
      g_sensor_data = calibrated_gyro_accel;
    } else {
      static uint32_t gyro_err_count = 0;
      gyro_err_count++;
      if (gyro_err_count <= 3 || (gyro_err_count % 50) == 0) {
        ESP_LOGW(TAG, "Gyro/accel read failed (total=%u): %s", (unsigned)gyro_err_count, esp_err_to_name(gyro_ret));
      }
    }

    // Read calibrated magnetometer (no retries; log source only).
    icm20948_mag_data_t calibrated_mag;
    esp_err_t mag_ret = icm20948_read_mag_calibrated(params->icm20948_dev_handle, &g_mag_bias, &calibrated_mag);
    if (mag_ret == ESP_OK) {
      g_mag_data = calibrated_mag;
    } else {
      static uint32_t mag_err_count = 0;
      mag_err_count++;
      if (mag_err_count <= 3 || (mag_err_count % 50) == 0) {
        ESP_LOGW(TAG, "Mag read failed (total=%u): %s", (unsigned)mag_err_count, esp_err_to_name(mag_ret));
      }
    }

    // Remap sensor axes into the robot frame for the current ICM20948
    // mounting: accel/gyro +X is forward, gyro +X is right-roll positive,
    // gyro +Y is nose-down, gyro +Z is CCW, mag +X is north/forward,
    // mag +Y is east/right, and mag +Z points down.
    float gx = g_sensor_data.gyro_x;
    float gy = -g_sensor_data.gyro_y;
    float gz = -g_sensor_data.gyro_z;

    float accx = -g_sensor_data.accel_x;
    float accy = g_sensor_data.accel_y;
    float accz = g_sensor_data.accel_z;

    float mx_cal = -g_mag_data.mag_y;
    float my_cal = g_mag_data.mag_x;
    float mz_cal = -g_mag_data.mag_z;

    // Magnetometer heading (diagnostic).
    float heading;
    heading = atan2f(mx_cal, my_cal) * (180.0f / M_PI);
    if (heading < 0)
      heading += 360.0f;
    ESP_LOGI("task sensor_fusion loop", "Magnetometer heading = %.2f deg", heading);

    static sensor_stream_mode_t prev_mode = MODE_MAG_ACCEL_GYRO;
    if (g_sensor_stream_mode != prev_mode) {
      if (g_sensor_stream_mode == MODE_ACCEL_GYRO) {
        fusion_set_gain(FUSION_GAIN_ACCEL_GYRO);
        fusion_reset();
        accel_gyro_seed_pending = 1;
        accel_gyro_seed_samples = 0;
        accel_gyro_seed_sum_sin = 0.0f;
        accel_gyro_seed_sum_cos = 0.0f;
        accel_gyro_seed_heading = heading;
      } else if (g_sensor_stream_mode == MODE_MAG_ACCEL) {
        fusion_set_gain(FUSION_GAIN_MAG_ACCEL);
        fusion_reset();
        accel_gyro_seed_pending = 0;
      } else if (g_sensor_stream_mode == MODE_MAG_ACCEL_GYRO) {
        fusion_set_gain(FUSION_GAIN_MAG_ACCEL_GYRO);
        fusion_reset();
        accel_gyro_seed_pending = 0;
      } else {
        fusion_set_gain(FUSION_GAIN_DEFAULT);
        accel_gyro_seed_pending = 0;
      }
      prev_mode = g_sensor_stream_mode;
    }

    // Adaptive dt calculation
    uint64_t now_us = esp_timer_get_time();
    float dt = (last_us == 0) ? (g_sensor_loop_delay_ms / 1000.0f) : (now_us - last_us) / 1e6f;
    last_us = now_us;

    // Convert dt to ms for logging (kept as comment to avoid unused var)
    // float dt_ms = dt * 1000.0f;

    // Update the selected fusion backend.
    if (g_sensor_stream_mode == MODE_ACCEL_GYRO) {
      if (accel_gyro_seed_pending && mag_ret == ESP_OK) {
        const float heading_rad = heading * ((float)M_PI / 180.0f);
        accel_gyro_seed_sum_sin += sinf(heading_rad);
        accel_gyro_seed_sum_cos += cosf(heading_rad);
        accel_gyro_seed_samples++;
        accel_gyro_seed_heading = atan2f(accel_gyro_seed_sum_sin, accel_gyro_seed_sum_cos) * (180.0f / (float)M_PI);
        if (accel_gyro_seed_heading < 0.0f) {
          accel_gyro_seed_heading += 360.0f;
        }

        if (accel_gyro_seed_samples >= accel_gyro_seed_target) {
          fusion_seed_imu_yaw(accel_gyro_seed_heading, accx, accy, accz);
          accel_gyro_seed_pending = 0;
        }
      }

      if (!accel_gyro_seed_pending) {
        fusion_update_imu(gx, gy, gz, accx, accy, accz, dt);
      }
    } else if (g_sensor_stream_mode == MODE_MAG_ACCEL) {
      fusion_update(0.0f, 0.0f, 0.0f, accx, accy, accz, mx_cal, my_cal, mz_cal, dt);
    } else if (g_sensor_stream_mode == MODE_MAG_ACCEL_GYRO) {
      fusion_update(gx, gy, gz, accx, accy, accz, mx_cal, my_cal, mz_cal, dt);
    }

    static float yaw_integrated_rad = 0.0f;

    if (dt > 0.0f) {
      yaw_integrated_rad += gz * dt;
    }
    float yaw_integrated_deg = fmodf(yaw_integrated_rad * (180.0f / M_PI), 360.0f);
    if (yaw_integrated_deg < 0)
      yaw_integrated_deg += 360.0f;

    // Get orientation
    float yaw, pitch, roll;
    if (g_sensor_stream_mode == MODE_MAG) {
      yaw = heading;
      pitch = 0.0f;
      roll = 0.0f;
    } else if (g_sensor_stream_mode == MODE_ACCEL_GYRO && accel_gyro_seed_pending) {
      yaw = accel_gyro_seed_heading;
      pitch = -atan2f(accx, sqrtf(accy * accy + accz * accz)) * (180.0f / (float)M_PI);
      roll = atan2f(accy, accz) * (180.0f / (float)M_PI);
    } else {
      fusion_get_orientation(&yaw, &pitch, &roll);
    }

    ESP_LOGI(TAG, "Euler: Yaw=%.2f deg. Pitch=%.2f deg. Roll=%.2f deg.", yaw, pitch, roll);

    if (g_sensor_stream_enabled && g_mqtt_client) {
      uint64_t now_us_stream = esp_timer_get_time();
      uint64_t period_us = (uint64_t)g_sensor_stream_period_ms * 1000ULL;
      if (period_us == 0) {
        period_us = 1000ULL;
      }
      if ((now_us_stream - last_stream_us) >= period_us) {
        char topic[64];
        char payload[160];
        snprintf(topic, sizeof(topic), "arena/%s/orientation", g_hostname);
        snprintf(payload, sizeof(payload), "{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f}", yaw, pitch, roll);
        esp_mqtt_client_publish(g_mqtt_client, topic, payload, 0, 0, 0);
        last_stream_us = now_us_stream;
      }
    }

    // Log all three together
    // ESP_LOGI(TAG, "sample= %d", sample_count);
    // ESP_LOGI(TAG, "dt= %.2f ms", dt_ms);
    // ESP_LOGI(TAG, "g_x = %.2f, g_y = %.2f, g_z = %.2f, a_x = %.2f, a_y = %.2f, a_z = %.2f",
    //                      gx, gy, gz, accx, accy, accz);
    // ESP_LOGI(TAG, "RawMagYaw= %.2f deg, GyroIntYaw= %.2f deg, FusedYaw= %.2f deg, (Pitch= %.2f deg, Roll= %.2f
    // deg)\n\n",
    //               heading, yaw_integrated_deg, yaw, pitch, roll);

    sample_count++;

    // Handle counted mode
    if (g_sensor_loop_mode == SENSOR_LOOP_COUNTED) {
      if (g_sensor_loop_iterations > 0) {
        g_sensor_loop_iterations--;
      }
      if (g_sensor_loop_iterations == 0) {
        g_sensor_loop_mode = SENSOR_LOOP_STOP;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(g_sensor_loop_delay_ms));
  }
} // task_sensor_fusion()

void app_main(void)
{
  /*
    Outline of app_main():
    1. Basic setup (LEDs, NVS, MAC, hostname)
    2. WiFi initialization
    3. UDP logging setup
    4. RGB LED ring setup
    5. Hall sensor initialization
    6. I2C device handles initialization
    7. MQTT client start
    8. IMU sensors setup and calibration
    9. Task creation
  */
  char *TAG = "sparkcore main";
  int rgb_led_ring_gpio_pin = RGB_LED_RING_GPIO_PIN;

// We call set_next_boot_to_factory() at the very beginning of app_main()
// to ensure that the next boot will use the factory partition.
#ifdef OTA_UPDATE_ENABLED
  ESP_LOGI(TAG, "OTA update enabled");
  set_next_boot_to_factory();
#endif // OTA_UPDATE_ENABLED

  initialize_esp32cam_leds();
  test_esp32cam_leds();
  if (ESP_OK != initialize_nvs()) {
    ESP_LOGE(TAG, "Failed to initialize NVS");
    indicate_error_esp32cam_red_led();
    return;
  }

  // Get MAC address and determine SparkNode ID before WiFi initialization
  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  g_sparknode_id = mac_to_id(mac);
  if (g_sparknode_id == 0x00) {
    ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
  } else {
    ESP_LOGI(TAG, "SparkNode ID is %d", g_sparknode_id);
  }

  // Prepare hostname based on SparkNode ID
  if (g_sparknode_id != 0x00) {
    snprintf(g_hostname, sizeof(g_hostname), "sparknode%02d", g_sparknode_id);
  } else {
    // Fallback to MAC address if SparkNode ID is not available
    snprintf(g_hostname, sizeof(g_hostname), "esp32cam-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }

  initialize_wifi_sta_with_hostname(WIFI_SSID, WIFI_PASS, g_hostname);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();
  indicate_success_esp32cam_flash_led();
  // int64_t *start_time = malloc(sizeof(int64_t));
  //*start_time = esp_timer_get_time();  // For measuring elapsed time
  get_chip_info();
  fflush(stdout);
  get_reset_reason();

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *)ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }

  // Get and print the assigned IP address
  esp_netif_ip_info_t ip_info;
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
    ESP_LOGI(TAG, "Assigned IP Address is " IPSTR, IP2STR(&ip_info.ip));
  }

  ESP_LOGI(TAG, "RGB LED ring control pin is %d", rgb_led_ring_gpio_pin);
  initialize_rgb_led_ring(&g_rgb_led_ring_handle, rgb_led_ring_gpio_pin, RGB_LED_RING_LED_COUNT);
  set_leds_from_debruijn_sequence_progressive(g_rgb_led_ring_handle, g_sparknode_id, 200);

  // Initialize hall effect sensors for wheel counting
  // Create separate configurations for each sensor
  hall_sensor_config_t sensor_1_config = HALL_SENSOR_DEFAULT_CONFIG(13); // Left wheel
  hall_sensor_config_t sensor_2_config = HALL_SENSOR_DEFAULT_CONFIG(14); // Right wheel

  // Configure for 2 magnets per wheel revolution (default is 4)
  // sensor_1_config.magnets_per_revolution = 2;
  // sensor_2_config.magnets_per_revolution = 2;

  esp_err_t hall_init_result = initialize_hall_sensors(&sensor_1_config, &sensor_2_config);
  if (hall_init_result == ESP_OK) {
    ESP_LOGI(TAG, "Hall sensors initialized successfully");
    ESP_LOGI(TAG, "Left wheel sensor: GPIO%d, Right wheel sensor: GPIO%d", sensor_1_config.sensor_gpio,
             sensor_2_config.sensor_gpio);
    ESP_LOGI(TAG, "Left wheel magnets: %d, Right wheel magnets: %d", sensor_1_config.magnets_per_revolution,
             sensor_2_config.magnets_per_revolution);
  } else {
    ESP_LOGE(TAG, "Failed to initialize hall sensors: %s", esp_err_to_name(hall_init_result));
  }

  // Start MQTT client
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker.address.uri = "mqtt://" MQTT_BROKER_IP_ADDR,
  };
  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, g_hostname);
  esp_mqtt_client_start(client);
  g_mqtt_client = client;
  ESP_LOGI(TAG, "MQTT client started");

  initialize_i2c_master(&g_i2c_bus_handle);
  i2c_scan(g_i2c_bus_handle);

  // Add left motor device to the I2C master bus and obtain device handle.
  i2c_device_config_t left_motor_dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = LEFT_MOTOR_I2C_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &left_motor_dev_cfg, &g_left_motor_dev_handle));
  initialize_motor(g_left_motor_dev_handle, I2C_MASTER_TIMEOUT_MS);

  // Add right motor device to the I2C master bus and obtain device handle.
  i2c_device_config_t right_motor_dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = RIGHT_MOTOR_I2C_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &right_motor_dev_cfg, &g_right_motor_dev_handle));
  initialize_motor(g_right_motor_dev_handle, I2C_MASTER_TIMEOUT_MS);

  // Detect IMU sensors
  bool icm20948_found = false;
  bool bmp388_found = false;

  for (int i = 1; i < 127; i++) {
    esp_err_t espRc = i2c_master_probe(g_i2c_bus_handle, i, -1);
    if (espRc == ESP_OK) {
      ESP_LOGI(TAG, "Found device at 0x%02x.", i);
      if (i == ICM20948_I2C_ADDR) {
        icm20948_found = true;
        ESP_LOGI(TAG, "ICM20948 detected.");
      }
      if (i == BMP388_I2C_ADDR) {
        bmp388_found = true;
        ESP_LOGI(TAG, "BMP388 detected.");
      }
    }
  }
    
  if (bmp388_found) {
    ESP_LOGI(TAG, "Initializing BMP388 Temperature and Pressure Sensor...");

    i2c_device_config_t bmp388_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP388_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
    };
    i2c_master_dev_handle_t bmp388_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &bmp388_dev_cfg, &bmp388_dev_handle));

    initialize_bmp388(bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
    bmp388_read_data(bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
  } else {
    ESP_LOGW(TAG, "BMP388 is not detected, skipping BMP388 initialization");
  }

  if (icm20948_found) {
    ESP_LOGI(TAG, "Initializing ICM20948, starting sensor stack...");

    i2c_device_config_t icm20948_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ICM20948_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
    };
    ESP_LOGI(TAG, "icm20948_dev_cfg struct created ");
    i2c_master_dev_handle_t icm20948_dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c_bus_handle, &icm20948_dev_cfg, &icm20948_dev_handle));

    ESP_LOGI(TAG, "Starting verification ");
    icm20948_verify_device(icm20948_dev_handle);
    icm20948_initialize(icm20948_dev_handle);

    // Assign to global so MQTT handler can use it
    g_icm20948_dev_handle = icm20948_dev_handle;

    ESP_LOGI(TAG, "Starting to control calibrations for mag and gyro...");
    // mag_calibration_t mag_bias;
    esp_err_t cal_status = load_mag_calibration_from_nvs(&g_mag_bias);
    bool valid_mag_cal = (cal_status == ESP_OK) && (g_mag_bias.version == 1 || g_mag_bias.version == 3);

    if (!valid_mag_cal) {
      if (cal_status == ESP_OK) {
        ESP_LOGW(TAG, "Unsupported magnetometer calibration version: %d. Using identity calibration.",
                 g_mag_bias.version);
      } else {
        ESP_LOGW(TAG, "No valid magnetometer calibration found in NVS (%s). Using identity calibration.",
                 esp_err_to_name(cal_status));
      }

      memset(&g_mag_bias, 0, sizeof(g_mag_bias));
      g_mag_bias.soft_iron_matrix[0][0] = 1.0f;
      g_mag_bias.soft_iron_matrix[1][1] = 1.0f;
      g_mag_bias.soft_iron_matrix[2][2] = 1.0f;
      g_mag_bias.version = 1;
    } else {
      ESP_LOGI(TAG, "Loaded magnetometer calibration from NVS.");
      ESP_LOGI(TAG, "Hard-Iron Bias: X=%.2f, Y=%.2f, Z=%.2f", g_mag_bias.hard_iron_bias[0],
               g_mag_bias.hard_iron_bias[1], g_mag_bias.hard_iron_bias[2]);
      ESP_LOGI(TAG, "Soft-Iron Matrix:");
      ESP_LOGI(TAG, "[%.3f %.3f %.3f]", g_mag_bias.soft_iron_matrix[0][0], g_mag_bias.soft_iron_matrix[0][1],
               g_mag_bias.soft_iron_matrix[0][2]);
      ESP_LOGI(TAG, "[%.3f %.3f %.3f]", g_mag_bias.soft_iron_matrix[1][0], g_mag_bias.soft_iron_matrix[1][1],
               g_mag_bias.soft_iron_matrix[1][2]);
      ESP_LOGI(TAG, "[%.3f %.3f %.3f]", g_mag_bias.soft_iron_matrix[2][0], g_mag_bias.soft_iron_matrix[2][1],
               g_mag_bias.soft_iron_matrix[2][2]);
      ESP_LOGI(TAG, "Mag Calibration Version: %d", g_mag_bias.version);
    }

    // Gyro bias data
    // Note: Using global g_sensor_data and g_mag_data (defined at top of file)
    icm20948_data_t local_gyro_bias = {0}; // Initialize to zero
    gyro_calibration_t gyro_cal;
    esp_err_t gyro_status = load_gyro_calibration_from_nvs(&gyro_cal);

    if (gyro_status == ESP_OK && gyro_cal.version == 1) {
      ESP_LOGI(TAG, "Loaded gyro calibration from NVS.");
      local_gyro_bias.gyro_x = gyro_cal.gyro_bias[0];
      local_gyro_bias.gyro_y = gyro_cal.gyro_bias[1];
      local_gyro_bias.gyro_z = gyro_cal.gyro_bias[2];
      ESP_LOGI(TAG, "Gyro Bias: X=%.2f Y=%.2f Z=%.2f", local_gyro_bias.gyro_x, local_gyro_bias.gyro_y,
               local_gyro_bias.gyro_z);
      ESP_LOGI(TAG, "Gyro Calibration Version: %d", gyro_cal.version);
    } else {
      ESP_LOGW(TAG, "Gyro calibration missing or unsupported. Recalibrating...");
      icm20948_calibrate_gyro_bias(icm20948_dev_handle, GYRO_NUM_CALIBRATION_SAMPLES);

      // The calibration function updates global g_gyro_bias (icm20948_data_t)
      // Copy values to our local gyro_bias_t structure
      extern icm20948_data_t g_gyro_bias; // Global bias updated by calibration function
      local_gyro_bias.gyro_x = g_gyro_bias.gyro_x;
      local_gyro_bias.gyro_y = g_gyro_bias.gyro_y;
      local_gyro_bias.gyro_z = g_gyro_bias.gyro_z;

      ESP_LOGI(TAG, "Gyro recalibration completed. Bias: X=%.2f Y=%.2f Z=%.2f", local_gyro_bias.gyro_x,
               local_gyro_bias.gyro_y, local_gyro_bias.gyro_z);
    }

    // Initialize the active fusion backend.
    fusion_init(FUSION_GAIN_DEFAULT);

    // Read one calibrated sample to initialize orientation
    icm20948_read_gyro_calibrated(icm20948_dev_handle, &local_gyro_bias, &g_sensor_data);
    icm20948_read_mag_calibrated(icm20948_dev_handle, &g_mag_bias, &g_mag_data);
    fusion_reset();

    // Prepare parameters for sensor fusion task
    static sensor_fusion_params_t sensor_params = {0};

    // Copy the device handle and calibration data
    sensor_params.icm20948_dev_handle = icm20948_dev_handle;
    sensor_params.mag_bias = g_mag_bias;
    sensor_params.gyro_bias = local_gyro_bias;

    // Create sensor fusion task
    if (pdPASS != xTaskCreate(&task_sensor_fusion, "task_sensor_fusion", 4096, &sensor_params, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create sensor fusion task");
    }
  } else {
    ESP_LOGW(TAG, "ICM20948 is not detected, skipping setup and sensor fusion task");
  }

  // Create tasks
  if (pdPASS != xTaskCreate(&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
    ESP_LOGE(TAG, "Failed to create esp32cam_blink_red_led task");
  }

  if (pdPASS != xTaskCreate(&task_udp_heartbeat, "task_udp_heartbeat", 2048, NULL, 5, NULL)) {
    ESP_LOGE(TAG, "Failed to create UDP heartbeat task");
  }
  ESP_LOGI(TAG, "app_main() completed");
} // app_main()
