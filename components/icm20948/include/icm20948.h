/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * icm20948.h - ICM20948 9-DOF IMU sensor driver header
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

#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#define I2C_MASTER_TIMEOUT_MS           1000    // Define I2C timeout for transactions (adjust as needed)



// ICM20948 User Bank 0
// ****************************************************************************************
#define ICM20948_WHO_AM_I_REG           0x00     // WHO_AM_I register address
#define ICM20948_WHO_AM_I_VAL           0xEA     // Expected WHO_AM_I value
#define ICM20948_USER_CTRL              0x03     // USER_CTRL register address
#define ICM20948_FIFO_EN_BIT            (1 << 6) // FIFO Enable bit in USER_CTRL
#define ICM20948_I2C_MST_RST_BIT        (1 << 1) // Bit 1 in USER_CTRL (Bank 0)

#define ICM20948_LP_CONFIG              0x05     // LP CONFIG register address
#define ICM20948_PWR_MGMT_1             0x06     // Power Management register1 address
#define ICM20948_PWR_MGMT_2             0x07     // Power Management register2 address
#define ICM20948_INT_PIN_CFG            0x0F     // INT_PIN_CFG register address

#define ICM20948_SINGLE_FIFO_PRIORITY_SEL    0x26 //Thanks Sparkfun

// Accelerometer Output Registers (User Bank 0)
#define ICM20948_ACCEL_XOUT_H           0x2D  // High byte of Accelerometer X-axis data
#define ICM20948_ACCEL_XOUT_L           0x2E  // Low byte of Accelerometer X-axis data
#define ICM20948_ACCEL_YOUT_H           0x2F  // High byte of Accelerometer Y-axis data
#define ICM20948_ACCEL_YOUT_L           0x30  // Low byte of Accelerometer Y-axis data
#define ICM20948_ACCEL_ZOUT_H           0x31  // High byte of Accelerometer Z-axis data
#define ICM20948_ACCEL_ZOUT_L           0x32  // Low byte of Accelerometer Z-axis data

// Gyroscope Output Registers (User Bank 0)
#define ICM20948_GYRO_XOUT_H            0x33  // High byte of Gyroscope X-axis data
#define ICM20948_GYRO_XOUT_L            0x34  // Low byte of Gyroscope X-axis data
#define ICM20948_GYRO_YOUT_H            0x35  // High byte of Gyroscope Y-axis data
#define ICM20948_GYRO_YOUT_L            0x36  // Low byte of Gyroscope Y-axis data
#define ICM20948_GYRO_ZOUT_H            0x37  // High byte of Gyroscope Z-axis data
#define ICM20948_GYRO_ZOUT_L            0x38  // Low byte of Gyroscope Z-axis data

#define ICM20948_EXT_SLV_SENS_DATA_00   0x3B    // External Sensor Data start address

// FIFO Related Registers (User Bank 0)
#define ICM20948_FIFO_EN_1              0x66  // Enable FIFO for external I2C sensors
#define ICM20948_FIFO_EN_2              0x67  // Enable FIFO for accelerometer, gyro, and temperature
#define ICM20948_FIFO_RST               0x68  // Reset FIFO buffer
#define ICM20948_FIFO_MODE              0x69  // Configure FIFO operating mode (stream/snapshot)
#define ICM20948_FIFO_COUNTH            0x70  // FIFO count high byte
#define ICM20948_FIFO_COUNTL            0x71  // FIFO count low byte
#define ICM20948_FIFO_R_W               0x72  // FIFO read/write register
#define ICM20948_HW_FIX_DISABLE         0x75  // #thanks Sparkfun 
#define ICM20948_FIFO_CFG               0x76  // FIFO configuration (interrupt status for each sensor)
#define ICM20948_REG_BANK_SEL           0x7F  // Register Bank Select address

// DMP Related Registers (User Bank 0) Not specified in ICM20948 Datasheet !!
#define ICM20948_MEM_START_ADDR         0x7C
#define ICM20948_MEM_R_W                0x7D
#define ICM20948_MEM_BANK_SEL           0x7E

//DMP MEMORY INFOS
#define DMP_MEM_BANK_SIZE               256
#define DMP_LOAD_START                  0x90
#define DMP_START_ADDRESS               0x1000  // It won't be used, DMP when enabled knows where to start!
#define DMP_MAX_WRITE                   16

// ICM20948 User Bank 2
// ****************************************************************************************
#define ICM20948_GYRO_SMPLRT_DIV        0x00    // Gyroscope Sample Rate Divider bits [7:0]
#define ICM20948_GYRO_CONFIG_1          0x01    // Gyroscope Configuration 1
#define ICM20948_GYRO_CONFIG_2          0x02    // Gyroscope Configuration 2

#define ICM20948_ACCEL_SMPLRT_DIV_1     0x10    // Gyroscope Sample Rate Divider1 bits [11:8]
#define ICM20948_ACCEL_SMPLRT_DIV_2     0x11    // Gyroscope Sample Rate Divider2 bits [7:0]

#define ICM20948_ACCEL_CONFIG           0x14    //Accel LP Filter and Full Scale settings
#define ICM20948_ACCEL_CONFIG_2         0x15    //Accel Self Test and Averaged Samples settings

#define ICM20948_PRGM_START_ADDRH       0x50
#define ICM20948_PRGM_START_ADDRL       0x51

// Add other Bank 2 registers if you use them (e.g., ACCEL_SMPLRT_DIV_1, ACCEL_CONFIG)

// ICM20948 User Bank 3
// ****************************************************************************************
#define ICM20948_I2C_MST_ODR_CONFIG     0x00    // I2C Master ODR Configuration
#define ICM20948_I2C_MST_CTRL           0x01    // I2C Master Control
#define ICM20948_I2C_MST_DELAY_CTRL     0x02    // I2C Master Delay Control
#define ICM20948_I2C_SLV0_ADDR          0x03    // I2C Slave 0 Address
#define ICM20948_I2C_SLV0_REG           0x04    // I2C Slave 0 Register
#define ICM20948_I2C_SLV0_CTRL          0x05    // I2C Slave 0 Control
#define ICM20948_I2C_SLV0_DO            0x06    // I2C Slave 0 Data Out
// Add other I2C Slave 1-4 addresses if you use them
#define ICM20948_I2C_SLV4_ADDR          0x13    // I2C Slave 4 Address
#define ICM20948_I2C_SLV4_REG           0x14    // I2C Slave 4 Register
#define ICM20948_I2C_SLV4_CTRL          0x15    // I2C Slave 4 Control
#define ICM20948_I2C_SLV4_DO            0x16    // I2C Slave 4 Data Out
#define ICM20948_I2C_SLV4_DI            0x17    // I2C Slave 4 Data In

// AK09916 Magnetometer Definitions
#define ICM20948_MAG_ADDRESS            0x0C    // Magnetometer I2C slave address
#define AK09916_WIA2_REG                0x01    // AK09916 WHO_AM_I (WIA2) register address
#define ICM20948_MAG_DATA_START         0x10    // AK09916 ST1 register address (start of data read)
#define AK09916_DATA_BYTES_TO_READ      9       // Number of bytes to read from AK09916 (ST1 to ST2)
#define AK09916_CNTL2_REG               0x31    // AK09916 Control 2 register address
#define AK09916_MODE_CONT_MEAS1         0x02    // AK09916 Continuous Measurement Mode 1 (~10 Hz)
#define AK09916_CNTL3_REG               0x32    // AK09916 Control 3 register address
#define AK09916_SRST_BIT                0x01    // AK09916 Soft Reset bit (bit 0 in CNTL3)
#define AK09916_SENSITIVITY             0.15f   // AK09916 Sensitivity in uT/LSB (16-bit mode)

// ICM20948 Bit Definitions

#define ICM20948_I2C_SLV0_DLY_EN        (1 << 0) // Bit 0 in I2C_MST_DELAY_CTRL (Bank 3)

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} icm20948_data_t;

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
} icm20948_mag_data_t;

typedef struct {
    float mx;
    float my;
    float mz;
} raw_mag_sample_t;

typedef struct {
    float gyro_bias[3];   // [bias_x, bias_y, bias_z]
    uint8_t version;      // for future compatibility
} gyro_calibration_t;

// Structure to hold magnetometer calibration results
typedef struct {
    float hard_iron_bias[3];       // [bias_x, bias_y, bias_z]
    float soft_iron_matrix[3][3]; // 3x3 matrix, for 2D this will be identity
    uint8_t version;              // format version for compatibility
} mag_calibration_t;
extern mag_calibration_t g_mag_bias;
extern icm20948_data_t g_gyro_bias;
extern bool g_mag_ready;

esp_err_t icm20948_set_bank(i2c_master_dev_handle_t dev_handle, uint8_t bank);
esp_err_t icm20948_verify_bank(i2c_master_dev_handle_t dev_handle, uint8_t expected_bank);
esp_err_t icm20948_read_register(i2c_master_dev_handle_t dev_handle, uint8_t bank, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t icm20948_modify_register(i2c_master_dev_handle_t dev_handle, uint8_t bank, uint8_t reg, uint8_t mask, uint8_t value);

void icm20948_verify_device(i2c_master_dev_handle_t dev_handle);
void icm20948_initialize(i2c_master_dev_handle_t dev_handle);

esp_err_t icm20948_read_mag_data(i2c_master_dev_handle_t dev_handle, icm20948_mag_data_t *mag_data);

esp_err_t collect_mag_calibration_data(i2c_master_dev_handle_t dev_handle, raw_mag_sample_t mag_samples[]);

esp_err_t icm20948_read_sensor_data(i2c_master_dev_handle_t dev_handle, icm20948_data_t *sensor_data);
void icm20948_calibrate_gyro_bias(i2c_master_dev_handle_t dev_handle, int samples);
esp_err_t store_gyro_calibration_to_nvs(const gyro_calibration_t *cal);
esp_err_t load_gyro_calibration_from_nvs(gyro_calibration_t *cal);

esp_err_t icm20948_load_dmp_firmware(i2c_master_dev_handle_t dev_handle, const uint8_t *dmp_image, size_t dmp_image_size, uint16_t start_address);
esp_err_t icm20948_fast_load_dmp_firmware(i2c_master_dev_handle_t dev_handle, const uint8_t *dmp_image, size_t dmp_image_size, uint16_t start_address); // <-- ADD THIS LINE

esp_err_t icm20948_read_mag_calibrated(i2c_master_dev_handle_t dev_handle, const mag_calibration_t *bias, icm20948_mag_data_t *mag_cal);
esp_err_t icm20948_read_gyro_calibrated(i2c_master_dev_handle_t dev_handle, const icm20948_data_t *gyro_bias, icm20948_data_t *gyro_cal);

// Compute compass heading in degrees (0-360) from calibrated magnetometer data
float icm20948_compute_heading(const icm20948_mag_data_t *mag);

void configure_dmp_sensor_scaling(i2c_master_dev_handle_t dev_handle);
void configure_dmp_compass_mounting_matrix(i2c_master_dev_handle_t dev_handle);
void set_compass_time_buffer(i2c_master_dev_handle_t dev_handle);
void set_sensor_output_data_rate(i2c_master_dev_handle_t dev_handle);

esp_err_t icm20948_check_dmp_data_ready(i2c_master_dev_handle_t dev_handle, bool *data_ready);

esp_err_t icm20948_configure_fifo(i2c_master_dev_handle_t dev_handle);
esp_err_t icm20948_monitor_fifo(i2c_master_dev_handle_t dev_handle, int iterations, int delay_ms);
esp_err_t icm20948_test_fifo(i2c_master_dev_handle_t dev_handle);
esp_err_t icm20948_initialize_dmp(i2c_master_dev_handle_t dev_handle);

esp_err_t  icm20948_read_dmp_data(i2c_master_dev_handle_t dev_handle, uint8_t *buffer, size_t len);

void parse_dmp_data_0x0188(uint8_t *dmp_packet, size_t packet_size); // For 9-axis+6-axis Quaternion+Calibrated Gyroscope
void parse_dmp_data_0x0100(uint8_t *dmp_packet, size_t packet_size); // For 9-axis Quaternion
void parse_dmp_data_0x0080(uint8_t *dmp_packet, size_t packet_size); // For 6-axis Quaternion
void parse_dmp_data_0x0008(uint8_t *dmp_packet, size_t packet_size); // For Calibrated Gyroscope
void parse_dmp_data_0x0004(uint8_t *dmp_packet, size_t packet_size); // Calibrated Accel
void parse_dmp_data_0x0088(uint8_t *dmp_packet, size_t packet_size); // 6-axis quaternion + calibrated gyro

extern uint8_t firmware_data[];
extern const uint8_t dmp_image[];    
extern const size_t dmp_image_size;  


// DMP Related Definitions  

#define DMP_GYRO_SF                     0x30
#define DMP_ACC_SCALE                   0xE0
#define DMP_CPASS_MTX_00                0x40
#define DMP_CPASS_MTX_01                0x44
#define DMP_CPASS_MTX_02                0x48
#define DMP_CPASS_MTX_10                0x4C
#define DMP_CPASS_MTX_11                0x50
#define DMP_CPASS_MTX_12                0x54
#define DMP_CPASS_MTX_20                0x58
#define DMP_CPASS_MTX_21                0x5C
#define DMP_CPASS_MTX_22                0x60
#define DMP_ODR_ACCEL                   0xBE
#define DMP_ODR_GYRO                    0xBA
#define DMP_ODR_CPASS                   0xB6

// Definitions for enabling DMP Quaternion Feature
#define DMP_FEAT_EN_REG_BANK            4   // Bank for DMP_FEATURE_MASK_2 (0x0400 is start of bank 4)
#define DMP_FEAT_EN_REG_ADDR            0x00 // Address for DMP_FEATURE_MASK_2 within its bank (0x0400 -> LSB 0x00)
#define DMP_FEAT_QUAT_EN_VAL_H          0x02 // High byte for 9-axis quaternion feature (from SparkFun's 0x0200)
#define DMP_FEAT_QUAT_EN_VAL_L          0x00 // Low byte


//Definitions for DMP Data Output Control
#define DMP_DATA_OUT_CTL_REG_BANK       2     //Bank for DATA_OUT_CTL1 and DATA_OUT_CTL2
#define DMP_DATA_OUT_CTL1_REG_ADDR      0x54 // Start address for DATA_OUT_CTL1 and DATA_OUT_CTL2



// ICM20948 User Bank 0 
// reference: http://jevois.org/doc/ICM20948__regs_8H_source.html
#define ICM20948_REG_DMP_INT_STATUS      0x18
#define ICM20948_BIT_WAKE_ON_MOTION_INT  0x08
#define ICM20948_BIT_MSG_DMP_INT         0x0002
#define ICM20948_BIT_MSG_DMP_INT_0       0x0100  // CI Command
#define ICM20948_BIT_MSG_DMP_INT_2       0x0200  // CIM Command - SMD
#define ICM20948_BIT_MSG_DMP_INT_3       0x0400  // CIM Command - Pedometer
#define ICM20948_BIT_MSG_DMP_INT_4       0x1000  // CIM Command - Pedometer binning
#define ICM20948_BIT_MSG_DMP_INT_5       0x2000  // CIM Command - Bring To See Gesture
#define ICM20948_BIT_MSG_DMP_INT_6       0x4000  // CIM Command - Look To See Gesture

#define ICM20948_REG_TEMP_CONFIG         0x53    //< Temperature Configuration register

// Bank 1 register map 

#define ICM20948_REG_XA_OFFSET_H         0x14    //< Acceleration sensor X-axis offset cancellation high byte  */
#define ICM20948_REG_XA_OFFSET_L         0x15    //< Acceleration sensor X-axis offset cancellation low byte   */
#define ICM20948_REG_YA_OFFSET_H         0x17    //< Acceleration sensor Y-axis offset cancellation high byte  */
#define ICM20948_REG_YA_OFFSET_L         0x18    //< Acceleration sensor Y-axis offset cancellation low byte   */
#define ICM20948_REG_ZA_OFFSET_H         0x1A    //< Acceleration sensor Z-axis offset cancellation high byte  */
#define ICM20948_REG_ZA_OFFSET_L         0x1B    //< Acceleration sensor Z-axis offset cancellation low byte   */
 
#define ICM20948_REG_TIMEBASE_CORR_PLL   0x28    // < PLL Timebase Correction register 


//Interrupt Registers
#define ICM20948_INT_ENABLE              0x10    // Interrupt Enable Register (Bank 0)
#define ICM20948_INT_ENABLE_DMP_BIT      (1 << 1) // DMP_INT_EN bit in INT_ENABLE

#define ICM20948_INT_STATUS              0x19    // INT_STATUS Register (Bank 0)
#define ICM20948_BIT_DMP_INT            (1 << 1) // DMP_INT bit in INT_STATUS 

#define MAG_NUM_CALIBRATION_SAMPLES 150 // MAG Calibration Number of samples
#define MAG_CALIBRATION_DELAY_MS    100  // MAG Calibration sample delay



#define GYRO_NUM_CALIBRATION_SAMPLES 150 // GYRO Calibration Number of samples
#define GYRO_CALIBRATION_DELAY_MS    100  // GYRO Calibration sample delay

esp_err_t estimate_bias_centroid(const raw_mag_sample_t *samples,
    int num_samples,
    mag_calibration_t *results);

// Function prototype for the 2D calibration
esp_err_t icm20948_calibrate_mag_2d(i2c_master_dev_handle_t dev_handle,
                                   raw_mag_sample_t mag_samples[],
                                   int num_samples,
                                   mag_calibration_t *calibration_results);

esp_err_t icm20948_calibrate_mag_3d(i2c_master_dev_handle_t dev_handle,
                                    raw_mag_sample_t mag_samples[],
                                    int num_samples,
                                    mag_calibration_t *calibration_results);                                   

esp_err_t store_mag_calibration_to_nvs(const mag_calibration_t *cal);

esp_err_t load_mag_calibration_from_nvs(mag_calibration_t *cal);


#endif // ICM20948_H

