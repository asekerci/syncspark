/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * sparknode_icm20948.c - IMU sensor node for SynchroSpark
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
 *
 * Important:
 * Do not flash this program to the device, it will be downloaded wirelessly by the OTA updater.
 * This program assumes that sparknode_ota_updater is already placed in the factory partition of
 * the device.
 */

#define OTA_UPDATE_ENABLED

/*
include <stdio.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "esp32_chip_info.h"
#include "syncspark_config.h"
#include "sys_utils.h"
//#include "rgb_led_ring.h"  // Now includes set_leds_from_debruijn_sequence()
#include "ota_utils.h"
#include "wifi_net.h"
#include "wifi_credentials.h"
#include "network_config.h"

*/



#include <stdio.h>
#include <string.h>
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp32_chip_info.h"
#include "syncspark_config.h"
#include "sys_utils.h"
#include "nvs_utils.h"

#include "nvs_flash.h"
#include "nvs.h"

//#include <nvs_flash.h>
//#include "rgb_led_ring.h"
#include "ota_utils.h"
#include "wifi_net.h"
#include "bmp388.h"
#include "icm20948.h"
#include "esp_pm.h"
#include "esp32_chip_info.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "esp_timer.h"
#include "madgwick_interface.h"
#include "MadgwickAHRS.h"

// Create your own file to define these macros:
// #define WIFI_SSID "your wifi ssid here"
// #define WIFI_PASS "your wifi password here"
#include "wifi_credentials.h"

// Contains definitions of DESTINATION_IP_ADDR, DESTINATION_UDP_PORT, 
// LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT, UDP_PACKET_SIZE,
// CHECKSUM_FILE_URL and BINARY_FILE_URL
#include "network_config.h" 

static const char *TAG = "main";
static uint8_t g_sparknode_id = 0; // Will be set after reading the MAC address

/*
void task_blink_esp32cam_red_led(void *pvParameter)
{
  while (1) {
    blink_esp32cam_red_led (200, 1200);	// Blink the red LED continuously
  }
} // task_blink_esp32cam_red_led()

void task_udp_heartbeat(void *pvParameter) {
  static uint32_t i = 1;
  while (1) {
    blink_esp32cam_flash_led(9, 100);
    blink_esp32cam_flash_led(9, 100);
    ESP_LOGI("UDP heartbeat", "(id %d) %" PRIu32, g_sparknode_id, i++);
    vTaskDelay(pdMS_TO_TICKS(10000)); // Every 10 seconds
    }
} // task_udp_heartbeat()
*/
i2c_master_bus_handle_t bus_handle;

void i2c_master_initialize() {
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  ESP_LOGI(TAG, "I2C Master initialized successfully");
}

void i2c_scan() {
  ESP_LOGI(TAG, "Starting I2C scan...");
  int devices_found = 0;
  for (int i = 1; i < 127; i++) {
      if (i2c_master_probe(bus_handle, i, -1) == ESP_OK) {
          ESP_LOGI(TAG, "Found device at 0x%02x", i);
          devices_found++;
      }
  }
  if (devices_found == 0) {
      ESP_LOGW(TAG, "No I2C devices detected!");
  } else {
      ESP_LOGI(TAG, "I2C scan complete. %d device(s) found.", devices_found);
  }
}



void app_main(void) 
{
  static const char *TAG = "sparknode_icm20948";

  // We call set_next_boot_to_factory() at the very beginning of app_main()
  // to ensure that the next boot will use the factory partition.
  #ifdef OTA_UPDATE_ENABLED
    ESP_LOGI(TAG, "OTA update enabled");
    set_next_boot_to_factory();
  #endif // OTA_UPDATE_ENABLED

  initialize_esp32cam_leds(); // Initialize GPIO pins for built-in LEDs
  test_esp32cam_leds();       // Blink the Flash and red LEDs
  if (ESP_OK != initialize_nvs()) {
        ESP_LOGE(TAG, "Failed to initialize NVS");
        indicate_error_esp32cam_red_led();  
        return;
  }
  initialize_wifi_sta(WIFI_SSID, WIFI_PASS);
  blink_esp32cam_flash_led(9, 0);
  initialize_udp_log(LOG_SERVER_IP_ADDR, LOG_SERVER_UDP_PORT);
  ESP_LOGI(TAG, "Redirecting/copying log messages over UDP...");
  redirect_log_to_udp();

  indicate_success_esp32cam_flash_led();
  int64_t *start_time = malloc(sizeof(int64_t)); // Allocate memory for start_time
  *start_time = esp_timer_get_time();            // For measuring elapsed time
  get_chip_info(); fflush(stdout);

  wifi_ap_record_t ap_info;
  if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    ESP_LOGI(TAG, "Connected to %s", (char *) ap_info.ssid);
    ESP_LOGI(TAG, "RSSI: %d dBm", ap_info.rssi);
  } else {
    ESP_LOGW(TAG, "Failed to get AP info");
  }

  uint8_t mac[6];
  esp_efuse_mac_get_default(mac);
  g_sparknode_id = mac_to_id(mac);
  if (g_sparknode_id == 0x00) {
      ESP_LOGE(TAG, "MAC address is not registered, cannot determine my SparkNode ID");
  } else {
      ESP_LOGI(TAG, "SparkNode ID is %d", g_sparknode_id);
  }
  /*
  if (pdPASS != xTaskCreate(&task_udp_heartbeat, "task_udp_heartbeat", 2048, NULL, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create UDP heartbeat task");
  } 

  if (pdPASS != xTaskCreate (&task_blink_esp32cam_red_led, "task_esp32cam_blink_red_led", 2048, NULL, 5, NULL)) {
      ESP_LOGE("app_main", "Failed to create esp32cam_blink_red_led task");
  }
    */

  i2c_master_initialize();
  i2c_scan();

  // BMP388 setup
  i2c_device_config_t bmp388_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BMP388_I2C_ADDR,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  i2c_master_dev_handle_t bmp388_dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bmp388_dev_cfg, &bmp388_dev_handle));

  initialize_bmp388(bmp388_dev_handle, I2C_MASTER_TIMEOUT_MS);
  bmp388_read_data(bmp388_dev_handle,I2C_MASTER_TIMEOUT_MS );


  // ICM20948 setup
  i2c_device_config_t icm20948_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ICM20948_ADDRESS,
    .scl_speed_hz = I2C_MASTER_FREQ1_HZ,
  };
  ESP_LOGI(TAG, "icm20948_dev_cfg struct created ");
  i2c_master_dev_handle_t icm20948_dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &icm20948_dev_cfg, &icm20948_dev_handle));
  
  ESP_LOGI(TAG, "Starting verification ");
  icm20948_verify_device(icm20948_dev_handle);
  ESP_LOGI(TAG, "The end of app_main program");
  icm20948_initialize(icm20948_dev_handle);

  ESP_LOGI(TAG, "Starting temporary magnetometer read loop...");

  // After initializing icm20948_dev_handle
  
  mag_calibration_t mag_bias;
  esp_err_t cal_status = load_mag_calibration_from_nvs(&mag_bias);

  if (cal_status == ESP_OK) {
      if (mag_bias.version != 1) {
          ESP_LOGW(TAG, "Unsupported magnetometer calibration version: %d. Recalibrating...", mag_bias.version);
          raw_mag_sample_t mag_samples[MAG_NUM_CALIBRATION_SAMPLES];
          collect_mag_calibration_data(icm20948_dev_handle, mag_samples);
          ESP_ERROR_CHECK(icm20948_calibrate_mag_2d(icm20948_dev_handle, mag_samples, MAG_NUM_CALIBRATION_SAMPLES, &mag_bias));
          mag_bias.version = 1;
          ESP_ERROR_CHECK(store_mag_calibration_to_nvs(&mag_bias));
      } else {
          ESP_LOGI(TAG, "Loaded magnetometer calibration from NVS.");
      }
  } else {
      ESP_LOGW(TAG, "No valid calibration found. Running calibration pipeline...");
      raw_mag_sample_t mag_samples[MAG_NUM_CALIBRATION_SAMPLES];
      collect_mag_calibration_data(icm20948_dev_handle, mag_samples);
      ESP_ERROR_CHECK(icm20948_calibrate_mag_2d(icm20948_dev_handle, mag_samples, MAG_NUM_CALIBRATION_SAMPLES, &mag_bias));
      mag_bias.version = 1;
      ESP_ERROR_CHECK(store_mag_calibration_to_nvs(&mag_bias));
  }

  ESP_LOGI(TAG, "Mag Bias: X=%.2f, Y=%.2f, Z=%.2f",
    mag_bias.hard_iron_bias[0],
    mag_bias.hard_iron_bias[1],
    mag_bias.hard_iron_bias[2],
    mag_bias.version);

  ESP_LOGI(TAG, "Mag Calibration Version: %d", mag_bias.version);
  
  // Sensor data structs
  icm20948_data_t sensor_data; // Gyro + Accel data
  icm20948_mag_data_t mag_data; // Mag data
   
  gyro_calibration_t gyro_cal;
  esp_err_t gyro_status = load_gyro_calibration_from_nvs(&gyro_cal);

  if (gyro_status == ESP_OK) {
      if (gyro_cal.version != 1) {
          ESP_LOGW(TAG, "Unsupported gyro calibration version: %d. Recalibrating...", gyro_cal.version);
          icm20948_calibrate_gyro_bias(icm20948_dev_handle, GYRO_NUM_CALIBRATION_SAMPLES);
      } else {
          gyro_bias.gyro_x = gyro_cal.gyro_bias[0];
          gyro_bias.gyro_y = gyro_cal.gyro_bias[1];
          gyro_bias.gyro_z = gyro_cal.gyro_bias[2];
          ESP_LOGI(TAG, "Loaded gyro bias from NVS: X=%.2f Y=%.2f Z=%.2f", 
                  gyro_bias.gyro_x, gyro_bias.gyro_y, gyro_bias.gyro_z);
          
                  ESP_LOGI(TAG, "Gyro Calibration Version: %d", gyro_cal.version);

      }
  } else {
      ESP_LOGW(TAG, "No gyro calibration found. Running calibration...");
      icm20948_calibrate_gyro_bias(icm20948_dev_handle, GYRO_NUM_CALIBRATION_SAMPLES);
  }


  // Initialize Madgwick filter
  madgwick_init(icm20948_dev_handle,5.0f); // 5 Hz
  
  //This part added by copilot for better settling
  //**********************************************************************

  // Artificial fixed magnetometer data for Option 4
  //float mx_cal = -9.90f;
  //float my_cal = 2.55f;
  //float mz_cal = 0.0 ;// 34.80f;


  // Read one sample to initialize orientation
  icm20948_read_sensor_data(icm20948_dev_handle, &sensor_data);
  icm20948_read_mag_data(icm20948_dev_handle, &mag_data);
  float mx0 = mag_data.mag_x - mag_bias.hard_iron_bias[0];  // X-axis
  float my0 = mag_data.mag_y - mag_bias.hard_iron_bias[1];  // Y-axis
   float mz0 = mag_data.mag_z - mag_bias.hard_iron_bias[2];
  //float mz_cal = mag_data.mag_z - mag_bias.hard_iron_bias[2];  // Apply Z-bias correction

  Madgwick_initFromAccMag(sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                          mx0, my0, mz0);

  //**********************************************************************

  ESP_LOGI(TAG, "\n\nStarting sensor fusion loop...\n");

  static int sampleCount = 0;

   while (1) {

      // Read sensors
      icm20948_read_sensor_data(icm20948_dev_handle, &sensor_data);
      icm20948_read_mag_data(icm20948_dev_handle, &mag_data);

      //Mag Calibrated Data Option 1
      //Apply hard-iron bias correction to X and Y axes
      //mx_cal = mag_data.mag_x - mag_bias.hard_iron_bias[0];  // X-axis
      //my_cal = mag_data.mag_y - mag_bias.hard_iron_bias[1];  // Y-axis
      //mz_cal = 0.0f; // Optional: zero Z to enforce 2D fusion

      //Mag Calibrated Data Option 2
      //Apply hard-iron bias correction to X and Y axes
      //float mx_cal = mag_data.mag_x - mag_bias.hard_iron_bias[0];  // X-axis
      //float my_cal = mag_data.mag_y - mag_bias.hard_iron_bias[1];  // Y-axis
      //float mz_cal = mag_data.mag_z; // Use the actual Z-axis magnetometer data

      //Mag Calibrated Data Option 3    
      //Apply hard-iron bias correction to X,Y and Z axes
      float mx_cal = mag_data.mag_x - mag_bias.hard_iron_bias[0];  // X-axis
      float my_cal = mag_data.mag_y - mag_bias.hard_iron_bias[1];  // Y-axis
      float mz_cal = mag_data.mag_z - mag_bias.hard_iron_bias[2];  // Apply Z-bias correction

      //Mag Calibrated Data Option 4    
      //Apply Artifical fixed data Mag [x: -9.90 uT, y: 2.55 uT, z: 34.80 uT]
      //float mx_cal = -9.90;  // X-axis
      //float my_cal = 2.55;   // Y-axis
      //float mz_cal = 0.0; //34.80;  // Z-axis

      //Gyro Calibrated Data Option 1
      // Correct the gyroscope readings with the bias before updating the filter
      float gx = sensor_data.gyro_x - gyro_bias.gyro_x;
      float gy = sensor_data.gyro_y - gyro_bias.gyro_y;
      float gz = sensor_data.gyro_z - gyro_bias.gyro_z;
      
      //Gyro Calibrated Data Option 2
      //Artifical gyro data
      //float gx = 0.0; 
      //float gy = 0.0; 
      //float gz = 0.0;

      //  Convert to radians/sec
      //gx *= (M_PI / 180.0f);
      //gy *= (M_PI / 180.0f);
      //gz *= (M_PI / 180.0f);

      //Accel Calibrated Data Option 1   
      float accx = sensor_data.accel_x;
      float accy = sensor_data.accel_y;
      float accz = sensor_data.accel_z;

      //Accel Calibrated Data Option 2
      //Artifical gyro data     
      //float accx = 0.0;
      //float accy = 0.0;
      //float accz = 1.0;

      ESP_LOGI(TAG,"Calib Accel [x: %.2f uT, y: %.2f uT, z: %.2f uT]", accx, accy, accz);
      ESP_LOGI(TAG,"Calib Gyro [x: %.2f uT, y: %.2f uT, z: %.2f uT]", gx, gy, gz);
      ESP_LOGI(TAG,"Calib Mag [x: %.2f uT, y: %.2f uT, z: %.2f uT]", mx_cal, my_cal, mz_cal);      
      float mag_norm = sqrtf(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal);
      ESP_LOGI(TAG, "Mag norm: %.2f uT", mag_norm);

      //Madgwick_initFromAccMag(sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
      //          mx0, my0, mz0);


      if (sampleCount < 26 || sampleCount % 100 == 0) {
          Madgwick_setBeta(0.1);
          ESP_LOGI(TAG, "**************Madgwick Beta 0.1*****************Magnetomer On *************************");
          madgwick_update(gx, gy, gz, accx, accy, accz, mx_cal, my_cal, mz_cal);  // Full fusion

      } else {
          Madgwick_setBeta(0.1f);
          //madgwick_update_imu(gx, gy, gz, accx, accy, accz);           // IMU-only
          madgwick_update(gx, gy, gz, accx, accy, accz, mx_cal, my_cal, mz_cal);  // Full fusion
          
          //Crude yaw integration (for diagnostic comparison)
          static uint64_t last_time_us = 0;
          uint64_t now_us = esp_timer_get_time();
          float dt = (now_us - last_time_us) / 1e6f;
          last_time_us = now_us;

          static float yaw_integrated_rad = 0.0f;
          yaw_integrated_rad += gz * dt;  // dt = 1 / sample rate
          float yaw_integrated_deg = fmodf(yaw_integrated_rad * (180.0f / M_PI), 360.0f);
          if (yaw_integrated_deg < 0) yaw_integrated_deg += 360.0f;
          ESP_LOGI(TAG, "Integrated Yaw = %.2f deg", yaw_integrated_deg);
          
      }

      // Get orientation
      float yaw, pitch, roll;
      madgwick_get_orientation(&yaw, &pitch, &roll);
      ESP_LOGI(TAG, "Orientation: Yaw =%.2f deg, gz =%.2f, Pitch=%.2f deg, Roll=%.2f deg, sampleCount=%d\n", yaw, gz, pitch, roll, sampleCount);
      
      //Log quaternion directly after update
      float q0, q1, q2, q3;
      madgwick_get_quaternion(&q0, &q1, &q2, &q3);
      ESP_LOGI(TAG, "Quaternion: W=%.4f X=%.4f Y=%.4f Z=%.4f", q0, q1, q2, q3);
  
      vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz update rate

      sampleCount++;
  }

  ESP_LOGI(TAG, "** The end of app_main program ****\n");



/*
 
void print_yaw_task(void *arg) {
  while (true) {
      float roll, pitch, yaw;
      madgwick_get_orientation(&roll, &pitch, &yaw);

      // Yaw clamped to [0� � 360�] for compass-style output
      if (yaw < 0) yaw += 360.0f;

      ESP_LOGI("YAW_TRACKER", "Calibrated Yaw: %.2f�", yaw);

      vTaskDelay(pdMS_TO_TICKS(200)); // Log every 200ms
  }
}

xTaskCreate(print_yaw_task, "yaw_logger", 2048, NULL, 5, NULL);
*/

} // app_main()

