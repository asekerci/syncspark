/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * icm20948.c - ICM20948 9-DOF IMU sensor driver implementation
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

#include "esp_log.h"
#include <string.h>  // Needed for memcmp()
#include <math.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h" // For portTICK_PERIOD_MS
#include "freertos/task.h" // For vTaskDelay
#include "esp_timer.h"
#include "esp_rom_sys.h" // Microsec delays
#include <stdio.h> // Include for printf and fflush
#include "nvs_flash.h"
#include "nvs.h"
#include "matrix_utils.h"

#include "icm20948.h"

static const char *TAG = "icm20948";
icm20948_data_t g_gyro_bias = {0};  // Store bias globally
mag_calibration_t g_mag_bias = {0};
bool g_mag_ready = false;

//Helper Functions

// Helper function to switch banks
esp_err_t icm20948_set_bank(i2c_master_dev_handle_t dev_handle, uint8_t bank) {
    uint8_t bank_val = (bank & 0x03) << 4; // Bank value is in bits 5:4
    uint8_t write_buf[] = {ICM20948_REG_BANK_SEL, bank_val};
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch to BANK_%d. Error: %s", bank, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Switched to BANK_%d.", bank); // Use Debug level for routine switches
    }
    return ret;
}
// Helper Function to read and verify the selected bank
esp_err_t icm20948_verify_bank(i2c_master_dev_handle_t dev_handle, uint8_t expected_bank) {
    if (!dev_handle) {
        ESP_LOGE(TAG, "Invalid I2C handle. Cannot verify bank.");
        return ESP_FAIL;
    }

    uint8_t reg_addr = ICM20948_REG_BANK_SEL; // Bank selection register
    uint8_t read_bank = 0;

    // Read the current bank setting
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &read_bank, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BANK_SEL register. Error: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // Extract bank number (Bits 5:4)
    read_bank = (read_bank >> 4) & 0x03;

    // Compare with expected bank
    if (read_bank != expected_bank) {
        ESP_LOGE(TAG, "Bank verification failed! Expected BANK_%d, but read BANK_%d", expected_bank, read_bank);
        return ESP_FAIL;
    }
    else {
    ESP_LOGI(TAG, "Bank verification successful. Current BANK: %d", read_bank);
    }

    return ESP_OK;

}
// Helper function to read a single register
esp_err_t icm20948_read_register(i2c_master_dev_handle_t dev_handle, uint8_t bank, uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret;
    uint8_t current_bank;

    // Read current bank to restore later
    ret = i2c_master_transmit_receive(dev_handle, (uint8_t[]){ICM20948_REG_BANK_SEL}, 1, &current_bank, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current bank for read operation. Error: %s", esp_err_to_name(ret));
        return ret;
    }
    current_bank = (current_bank >> 4) & 0x03; // Extract current bank

    // Switch to target bank
    ret = icm20948_set_bank(dev_handle, bank);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to switch to target bank %d for read. Error: %s", bank, esp_err_to_name(ret));
        return ret;
    }

    // Read register value
    ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X in bank %d. Error: %s", reg_addr, bank, esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "Read reg 0x%02X in bank %d: 0x%02X", reg_addr, bank, *data);
    }

    // Restore original bank
    esp_err_t restore_ret = icm20948_set_bank(dev_handle, current_bank);
    if (restore_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restore original bank %d after read. Error: %s", current_bank, esp_err_to_name(restore_ret));
    }
    return ret;
}
// Helper function to modify a register by changing only necessary bits
esp_err_t icm20948_modify_register(i2c_master_dev_handle_t dev_handle, uint8_t bank, uint8_t reg, uint8_t mask, uint8_t value) {
    esp_err_t ret;
    uint8_t current_bank;

    // Step 1: Read and preserve current bank
    uint8_t bank_read_buf[1];
    ret = i2c_master_transmit_receive(dev_handle, (uint8_t[]){ICM20948_REG_BANK_SEL}, 1, bank_read_buf, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current bank.");
        return ret;
    }
    current_bank = bank_read_buf[0] >> 4;  // Extract active bank

    // Step 2: Switch to target bank (only if different)
    if (current_bank != bank) {
        uint8_t bank_sel_buf[] = {ICM20948_REG_BANK_SEL, (bank << 4)};
        ret = i2c_master_transmit(dev_handle, bank_sel_buf, sizeof(bank_sel_buf), I2C_MASTER_TIMEOUT_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to switch to bank %d.", bank);
            return ret;
        }
    }

    // Step 3: Read current register value
    uint8_t reg_value;
    uint8_t reg_buf[] = {reg};
    ret = i2c_master_transmit_receive(dev_handle, reg_buf, 1, &reg_value, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X in bank %d.", reg, bank);
        return ret;
    }

    // Step 4: Modify only necessary bits (preserve other bits)
    reg_value = (reg_value & ~mask) | (value & mask);

    // Step 5: Write back modified register value
    uint8_t write_buf[] = {reg, reg_value};
    ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write modified register 0x%02X in bank %d.", reg, bank);
        return ret;
    }

    // Step 6: Restore original bank (if changed)
    if (current_bank != bank) {
        uint8_t restore_bank_buf[] = {ICM20948_REG_BANK_SEL, (current_bank << 4)};
        ret = i2c_master_transmit(dev_handle, restore_bank_buf, sizeof(restore_bank_buf), I2C_MASTER_TIMEOUT_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to restore original bank %d.", current_bank);
            return ret;
        }
    }

    ESP_LOGD(TAG, "Register 0x%02X in bank %d modified successfully. Restored bank %d.", reg, bank, current_bank);
    return ESP_OK;
}

//Functions called from app_main

//Called from app_main
void icm20948_verify_device(i2c_master_dev_handle_t dev_handle) {
    // Ensure BANK 0 before verifying
    if (icm20948_set_bank(dev_handle, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set bank 0 before verify.");
        return;
    }

    uint8_t reg_addr = ICM20948_WHO_AM_I_REG;
    uint8_t who_am_i = 0;
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &who_am_i, 1, 1000 / portTICK_PERIOD_MS);

    if (ret == ESP_OK && who_am_i == ICM20948_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "ICM20948 detected successfully. WHO_AM_I: 0x%02X", who_am_i);
    } else if (ret == ESP_OK) {
        ESP_LOGE(TAG, "ICM20948 verification failed. WHO_AM_I: 0x%02X (Expected: 0x%02X)", who_am_i, ICM20948_WHO_AM_I_VAL);
    } else {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register. Error: %s", esp_err_to_name(ret));
    }
}
//Called from icm20948_initialize Step 10
esp_err_t icm20948_initialize_magnetometer(i2c_master_dev_handle_t dev_handle) {
    esp_err_t ret;
    bool mag_init_success = false;
    int tries = 0;
    const int maxTries = 5;

    // Trigger Soft Reset for Magnetometer using AUX I2C Master
    if (icm20948_set_bank(dev_handle, 3) == ESP_OK) {
        uint8_t slv4_addr_srst_data[] = {ICM20948_I2C_SLV4_ADDR, ICM20948_MAG_ADDRESS};
        uint8_t slv4_reg_srst_data[] = {ICM20948_I2C_SLV4_REG, AK09916_CNTL3_REG};
        uint8_t slv4_do_srst_data[] = {ICM20948_I2C_SLV4_DO, AK09916_SRST_BIT};
        uint8_t slv4_ctrl_trigger_data[] = {ICM20948_I2C_SLV4_CTRL, 0x80}; // Enable transaction

        ret = i2c_master_transmit(dev_handle, slv4_addr_srst_data, sizeof(slv4_addr_srst_data), 1000 / portTICK_PERIOD_MS);
        ret |= i2c_master_transmit(dev_handle, slv4_reg_srst_data, sizeof(slv4_reg_srst_data), 1000 / portTICK_PERIOD_MS);
        ret |= i2c_master_transmit(dev_handle, slv4_do_srst_data, sizeof(slv4_do_srst_data), 1000 / portTICK_PERIOD_MS);
        ret |= i2c_master_transmit(dev_handle, slv4_ctrl_trigger_data, sizeof(slv4_ctrl_trigger_data), 1000 / portTICK_PERIOD_MS);

        if (ret == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Allow reset to complete
            ESP_LOGI(TAG, " Magnetometer Soft Reset triggered via AUX I2C Master.");
        } else {
            ESP_LOGE(TAG, "Failed to trigger Magnetometer Soft Reset.");
            return ret;
        }
    }

    // Retry WHO_AM_I Verification Loop
    while (tries < maxTries && !mag_init_success) {
        if (icm20948_set_bank(dev_handle, 3) != ESP_OK) return ESP_FAIL;

        uint8_t slv4_addr_whoami_data[] = {ICM20948_I2C_SLV4_ADDR, ICM20948_MAG_ADDRESS | 0x80}; // Read mode
        uint8_t slv4_reg_whoami_data[] = {ICM20948_I2C_SLV4_REG, AK09916_WIA2_REG};
        uint8_t slv4_ctrl_whoami_trigger[] = {ICM20948_I2C_SLV4_CTRL, 0x80}; // Enable transaction

        ret = i2c_master_transmit(dev_handle, slv4_addr_whoami_data, sizeof(slv4_addr_whoami_data), 1000 / portTICK_PERIOD_MS);
        ret |= i2c_master_transmit(dev_handle, slv4_reg_whoami_data, sizeof(slv4_reg_whoami_data), 1000 / portTICK_PERIOD_MS);
        ret |= i2c_master_transmit(dev_handle, slv4_ctrl_whoami_trigger, sizeof(slv4_ctrl_whoami_trigger), 1000 / portTICK_PERIOD_MS);

        vTaskDelay(pdMS_TO_TICKS(10)); // Allow read operation

        uint8_t mag_who_am_i_val = 0;
        uint8_t slv4_di_reg = ICM20948_I2C_SLV4_DI;
        ret = i2c_master_transmit_receive(dev_handle, &slv4_di_reg, 1, &mag_who_am_i_val, 1, 1000 / portTICK_PERIOD_MS);

        if (ret == ESP_OK && mag_who_am_i_val == 0x09) {
            ESP_LOGI(TAG, " Magnetometer WHO_AM_I verified as 0x09 on try 1..");
            mag_init_success = true;
            break;
        } else {
            ESP_LOGE(TAG, " Magnetometer WHO_AM_I verification failed on try %d. Read 0x%02X", tries + 1, mag_who_am_i_val);
        }

        tries++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!mag_init_success) {
        ESP_LOGE(TAG, " Magnetometer initialization failed after %d tries.", maxTries);
        return ESP_FAIL;
    }

    return ESP_OK;
}
//Called from app_main
void icm20948_initialize(i2c_master_dev_handle_t dev_handle) {
    ESP_LOGI(TAG, "ICM20948 Sensor initialization started");
    g_mag_ready = false;
    if (dev_handle == NULL) {
        ESP_LOGE(TAG, "Device handle is NULL!");
        return;
    }

    esp_err_t ret;
    bool mag_init_ok = true;

    // Step 1 --- Soft Reset (Recommended) ---
    if (icm20948_set_bank(dev_handle, 0) != ESP_OK) return ; // Switch to BANK 0 first
    // Set DEVICE_RESET bit (bit 7) in PWR_MGMT_1 (Bank 0, 0x06)
    uint8_t pwr_mgmt_1_reset[] = {ICM20948_PWR_MGMT_1, 0x80};
    ret = i2c_master_transmit(dev_handle, pwr_mgmt_1_reset, sizeof(pwr_mgmt_1_reset), 100 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 1 - Soft Reset is successful");
    } else {
        ESP_LOGE(TAG, "Step 1 - Failed to Soft Reset ICM20948. Error: %s", esp_err_to_name(ret));
        // Continue? Might be critical failure depending on requirements
    }

    // Step 2 --- Soft Reset (Recommended) ---
    vTaskDelay(100 / portTICK_PERIOD_MS); // datasheet recommends > 100ms after reset
    ESP_LOGI(TAG, "Step 2 - Wait for 100ms after Soft Reset");

    // Step 3 --- Initial Power Management and Clock Source ---
    // Set clock source to Auto (PLL if ready) and disable sleep mode after reset
    uint8_t pwr_mgmt_1_cfg[] = {ICM20948_PWR_MGMT_1, 0x01}; // CLKSEL = 001 (Auto)
    ret = i2c_master_transmit(dev_handle, pwr_mgmt_1_cfg, sizeof(pwr_mgmt_1_cfg), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 3 - Internal system clock sources is set to Auto 001.");
        //Auto 001 = PLL with MEMS Gyroscope Oscillator

    } else {
        ESP_LOGE(TAG, "Step 3 - Failed to set sensor clock source to Auto. Error: %s", esp_err_to_name(ret));
        // Continue? Might be critical failure depending on requirements
    }
    
    // Step 4 --- Ensures stable operation after changing clock settings ---
     vTaskDelay(10 / portTICK_PERIOD_MS);
     ESP_LOGI(TAG, "Step 4 - Wait 10ms for PLL stabilization ");

    // Step 5 --- Enable I2C Master Mode and Disable Bypass (Still BANK 0)---
    uint8_t user_ctrl_data[] = {ICM20948_USER_CTRL, 0x20}; // Set I2C_MST_EN = 1
    ret = i2c_master_transmit(dev_handle, user_ctrl_data, sizeof(user_ctrl_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 5 - AUX I2C Master Mode enabled.");
    } else {
        ESP_LOGE(TAG, "Step 5 - Failed to enable AUX I2C Master mode. Error: %s", esp_err_to_name(ret));
        // Continue? Might be critical failure depending on requirements
    }

    // Step 6 --- Disable Bypass When I2C Master Disabled
    uint8_t int_pin_cfg_data[] = {ICM20948_INT_PIN_CFG, 0x00}; // Ensure BYPASS_EN = 0
    ret = i2c_master_transmit(dev_handle, int_pin_cfg_data, sizeof(int_pin_cfg_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 6 - AUX I2C Bypass Mode disabled.");
    } else {
        ESP_LOGE(TAG, "Step 6 - Failed to disable AUX I2C Bypass Mode. Error: %s", esp_err_to_name(ret));
        // Continue
    }

    // Step 7 --- Enable Accel and Gyro (Still BANK 0) ---
    uint8_t pwr_mgmt_data_2[] = {ICM20948_PWR_MGMT_2, 0x00}; // Enable all axes
    ret = i2c_master_transmit(dev_handle, pwr_mgmt_data_2, sizeof(pwr_mgmt_data_2), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 7 - All Gyro and Accel Axes are enabled.");
    } else {
        ESP_LOGE(TAG, "Step 7 - Failed to configure power mode 2. Error: %s", esp_err_to_name(ret));
        // Continue
    }

    // ***** NEW3 STEP: Configure Accel/Gyro Digital Low Pass Filters and Full Scale*****
    ESP_LOGI(TAG, "Step 7.5 - Configuring Accel/Gyro Digital Low Pass Filters and Full Scale with different values");

    // --- Gyroscope Configuration (+/- 250 dps) ---
    // We want to set:
    // - DLPF_CFG [5:3] = 3 (0b011)
    // - FS_SEL   [2:1] = 0 (0b00 for +/- 250dps)
    // - FCHOICE  [0]   = 1 (to enable the DLPF)
    // The combined value is 0b0011001, which is 0x19.
    uint8_t gyro_config_mask = 0x3F;
    uint8_t gyro_config_value = 0x19;

    // --- Accelerometer Configuration (+/- 2g) ---
    // We want to set:
    // - DLPF_CFG [5:3] = 3 (0b011)
    // - FS_SEL   [2:1] = 0 (0b00 for +/- 2g)
    // - FCHOICE  [0]   = 1 (to enable the DLPF)
    // The combined value is 0b0011001, which is 0x19.
    uint8_t accel_config_mask = 0x3F;
    uint8_t accel_config_value = 0x19;


    // Modify Gyroscope configuration in BANK 2
    ret = icm20948_modify_register(dev_handle, 2, ICM20948_GYRO_CONFIG_1, gyro_config_mask, gyro_config_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GYRO_CONFIG_1. Error: %s", esp_err_to_name(ret));
    }

    // Modify Accelerometer configuration in BANK 2 with its own unique value
    ret = icm20948_modify_register(dev_handle, 2, ICM20948_ACCEL_CONFIG, accel_config_mask, accel_config_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ACCEL_CONFIG. Error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, " Accel/Gyro DLPF enabled successfully with different configurations.");
    }
    // ***** END OF NEW STEP *****

    /*
    // ***** NEW2 STEP: Configure Accel/Gyro Digital Low Pass Filters and Full Scale*****
    ESP_LOGI(TAG, "Step 7.5 - Configuring Accel/Gyro c");
    if (icm20948_set_bank(dev_handle, 2) != ESP_OK) return; // Switch to BANK 2

    // Set Gyro DLPF to 51.2 Hz (DLPFCFG = 3), and enable it (FCHOICE = 1)
    // GYRO_CONFIG_1 = 0b00011001 = 0x0D (DLPFCFG=11, FS_SEL=00, FCHOICE=1)
    // We will set DLPFCFG to a mid-range value like 3 (51.2Hz BW) and FS_SEL to 250dps (00)  
    uint8_t gyro_config_data[] = {ICM20948_GYRO_CONFIG_1, 0x37}; 
    ret = i2c_master_transmit(dev_handle, gyro_config_data, sizeof(gyro_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GYRO_CONFIG_1. Error: %s", esp_err_to_name(ret));
    }

    // Set Accel DLPF to 50.4 Hz (DLPFCFG = 3), and enable it (FCHOICE = 1)
    // ACCEL_CONFIG = 0b00011001 = 0x0D (DLPFCFG=11, FS_SEL=00, FCHOICE=1)
    // We will set DLPFCFG to a mid-range value like 3 (50.4Hz BW) and FS_SEL to +/-2g (00) 
    uint8_t accel_config_data[] = {ICM20948_ACCEL_CONFIG, 0x19};
    ret = i2c_master_transmit(dev_handle, accel_config_data, sizeof(accel_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ACCEL_CONFIG. Error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, " Accel/Gyro DLPF enabled successfully.");
    }
    // ***** END OF NEW STEP *****
    */
    /*
    // --- NEW1 STEP 7.5: Manually Set Sensor Range and DLPF for Lowest Sensitivity ---
    ESP_LOGI(TAG, "Step 7.5 - Setting Accel/Gyro range to lowest sensitivity");
    if (icm20948_set_bank(dev_handle, 2) != ESP_OK) return; // Switch to BANK 2

    // Value 0x1F = FS_SEL=11 (+/- 2000dps / +/- 16g) & DLPFCFG=3 (~51Hz)
    uint8_t config_value = 0x1F; 
    
    uint8_t gyro_config_data[] = {ICM20948_GYRO_CONFIG_1, config_value}; 
    ret = i2c_master_transmit(dev_handle, gyro_config_data, sizeof(gyro_config_data), I2C_MASTER_TIMEOUT_MS);
    
    uint8_t accel_config_data[] = {ICM20948_ACCEL_CONFIG, config_value};
    ret |= i2c_master_transmit(dev_handle, accel_config_data, sizeof(accel_config_data), I2C_MASTER_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set new range/DLPF config.");
    }
    // --- END OF NEW STEP ---
    */

    // Step 8 Configure Gyroscope ODR  (1100 / (1 + GYRO_SMPLRT_DIV))
    #define GYRO_SMPLRT_DIV  4 // Increase to 225Hz, old value 112 for around 10Hz.
    if (icm20948_set_bank(dev_handle, 2) != ESP_OK) return; // Switch to BANK 2
    uint8_t gyro_smplrt_div_data[] = {ICM20948_GYRO_SMPLRT_DIV, GYRO_SMPLRT_DIV};
    ret = i2c_master_transmit(dev_handle, gyro_smplrt_div_data, sizeof(gyro_smplrt_div_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Step 8 - Failed to configure GYRO_SMPLRT_DIV. Error: %s", esp_err_to_name(ret));
        // Continue, but note error
    } else {
        ESP_LOGI(TAG, "Step 8 - Gyroscope ODR configured to 225 Hz");
    }

    // Step 9 Configure Accelerometer ODR  (1125 / (1 + ACCEL_SMPLRT_DIV_2)) (Still BANK 2)
    #define ACCEL_SMPLRT_DIV_2  4 // Increase to 225Hz,old value 113 for around 10Hz.
    // ICM20948_ACCEL_SMPLRT_DIV_1 [11:8] high byte is by default 0
        uint8_t accel_smplrt_div_data[] = {ICM20948_ACCEL_SMPLRT_DIV_2, ACCEL_SMPLRT_DIV_2};
    ret = i2c_master_transmit(dev_handle, accel_smplrt_div_data, sizeof(accel_smplrt_div_data), 1000 / portTICK_PERIOD_MS);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 9 - Accelerometer ODR configured to 225 Hz");
    } else {
        ESP_LOGE(TAG, "Step 9 - Failed to configure ACCEL_SMPLRT_DIV. Error: %s", esp_err_to_name(ret));
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to ensure write is processed

    ESP_LOGI(TAG, "Step 9.5 - ODR Read-Back Verification");
    uint8_t gyro_div_val = 0;
    uint8_t accel_div_val = 0;
    
    // Read back the registers (they are still in BANK 2)
    icm20948_read_register(dev_handle, 2, ICM20948_GYRO_SMPLRT_DIV, &gyro_div_val, 1);
    icm20948_read_register(dev_handle, 2, ICM20948_ACCEL_SMPLRT_DIV_2, &accel_div_val, 1);

    ESP_LOGI(TAG, " ODR Read-Back Verification: GYRO_SMPLRT_DIV = %d, ACCEL_SMPLRT_DIV_2 = %d", gyro_div_val, accel_div_val);

    if (gyro_div_val != 4 || accel_div_val != 4) {
        ESP_LOGE(TAG, "FATAL: ODR setting did not stick! Halting.");
        while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); } // Halt execution
    }

    // Step 10 Magnetometer(AK09916) Initialization
    ESP_LOGI(TAG, "Step 10 - Magnetometer(AK09916) initialization started");
    ret = icm20948_initialize_magnetometer(dev_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, " Magnetometer initialization successful.");
    } else {
        ESP_LOGE(TAG, " Magnetometer initialization failed.");
        mag_init_ok = false;
    }

    // Step 11 --- Sets up automatic data fetching from magnetometer ---
    if (icm20948_set_bank(dev_handle, 3) != ESP_OK) return; // Switch to BANK 3
    // Configure I2C Master Control (Clock speed ~304,94kHz, non-stop between reads)
    uint8_t i2c_mst_ctrl_data[] = {ICM20948_I2C_MST_CTRL, 0x08 | (1 << 4)};
    ret = i2c_master_transmit(dev_handle, i2c_mst_ctrl_data, sizeof(i2c_mst_ctrl_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Step 11 - Failed to configure AUX I2C Master control. Error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Step 11 - AUX I2C Master control configured.");
    }

    // Step 12 Configure I2C Master ODR 1100/(2^((I2C_MST_ODR_CONFIG[3:0])) ) (Still in BANK 3)
    // When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR. 
    // If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR
    #define I2C_MST_ODR_CONFIG 7
    uint8_t i2c_mst_odr_config_data[] = {ICM20948_I2C_MST_ODR_CONFIG, I2C_MST_ODR_CONFIG};
    ret = i2c_master_transmit(dev_handle, i2c_mst_odr_config_data, sizeof(i2c_mst_odr_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Step 12 - Failed to configure I2C Master ODR. Error: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Step 12 - AUX I2C Master ODR set to ~8.59 Hz to match magnetometer update cycle.");
    }


    // Step 13 Enable duty-cycled mode for I2C Master, Accel, and Gyro for power save
    // Use after setting ODR
    // *** MODIFICATION: Comment out this section to disable duty-cycled mode ***
    //******************************************************************************************************
    /*
    uint8_t lp_config_data[] = {ICM20948_LP_CONFIG, 0x70};
    ret = i2c_master_transmit(dev_handle, lp_config_data, sizeof(lp_config_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 13 - Duty-Cycled Mode enabled.");
    } else {
        ESP_LOGE(TAG, "Step 13 - Failed to enable Duty-Cycled Mode. Error: %s", esp_err_to_name(ret));
    }
    */
    //******************************************************************************************************

    // Step 14 Configure I2C Slave 0: Read 9 bytes from Magnetometer starting at 0x10 (Still in BANK 3)
    uint8_t i2c_slv0_addr_data[] = {ICM20948_I2C_SLV0_ADDR, ICM20948_MAG_ADDRESS | 0x80}; // Mag Addr + Read flag
    ret = i2c_master_transmit(dev_handle, i2c_slv0_addr_data, sizeof(i2c_slv0_addr_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 14 - AUX I2C Slave 0 address set for Magnetometer read.");
        uint8_t i2c_slv0_reg_data[] = {ICM20948_I2C_SLV0_REG, ICM20948_MAG_DATA_START}; // Start reg 0x10
        ret = i2c_master_transmit(dev_handle, i2c_slv0_reg_data, sizeof(i2c_slv0_reg_data), 1000 / portTICK_PERIOD_MS);
    }
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Step 14 - AUX I2C Slave 0 register set to Magnetometer data start (0x10).");
        uint8_t i2c_slv0_ctrl_data[] = {ICM20948_I2C_SLV0_CTRL, AK09916_DATA_BYTES_TO_READ | (1 << 7)}; // Read 9 bytes + Enable
        ret = i2c_master_transmit(dev_handle, i2c_slv0_ctrl_data, sizeof(i2c_slv0_ctrl_data), 1000 / portTICK_PERIOD_MS);
    }
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Step 14 - Failed to configure AUX I2C Slave 0. Error: %s", esp_err_to_name(ret));
         g_mag_ready = false;
         return; // Critical failure
    } else {
        ESP_LOGI(TAG, "Step 14 - AUX I2C Slave 0 control set to read %d bytes and enabled.", AK09916_DATA_BYTES_TO_READ);
    }

    // Step 15 --- Configure I2C Master Delay Control used for magnetomter fresh reads(Bank 3) ---
    // *** MODIFICATION: Comment out this section to disable delayed sensing ***
    //******************************************************************************************************
    uint8_t i2c_mst_delay_ctrl_data[] = {ICM20948_I2C_MST_DELAY_CTRL, ICM20948_I2C_SLV0_DLY_EN}; // Enable delay for SLV0
    ret = i2c_master_transmit(dev_handle, i2c_mst_delay_ctrl_data, sizeof(i2c_mst_delay_ctrl_data), 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Step 15 - Failed to configure I2C Master Delay Control. Error: %s", esp_err_to_name(ret));
        // Continue, but note the error
    } else {
        ESP_LOGI(TAG, "Step 15 - I2C Master Delayed Sensing enabled for Slave 0."); // This log will now be absent
    }
    //******************************************************************************************************
 
    // Step 16 --- Initialize Magnetometer (AK09916) Operating Mode (Still BANK 3)---
    // Use SLV4 to write AK09916_CNTL2 (0x31) to set Continuous Measurement Mode 1 (0x02)
    uint8_t slv4_addr_mode_data[] = {ICM20948_I2C_SLV4_ADDR, ICM20948_MAG_ADDRESS};
    uint8_t slv4_reg_mode_data[] = {ICM20948_I2C_SLV4_REG, AK09916_CNTL2_REG};
    uint8_t slv4_do_mode_data[] = {ICM20948_I2C_SLV4_DO, AK09916_MODE_CONT_MEAS1};
    uint8_t slv4_ctrl_mode_trigger[] = {ICM20948_I2C_SLV4_CTRL, 0x80}; // Enable transaction

    ret = i2c_master_transmit(dev_handle, slv4_addr_mode_data, sizeof(slv4_addr_mode_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) ret = i2c_master_transmit(dev_handle, slv4_reg_mode_data, sizeof(slv4_reg_mode_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) ret = i2c_master_transmit(dev_handle, slv4_do_mode_data, sizeof(slv4_do_mode_data), 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) ret = i2c_master_transmit(dev_handle, slv4_ctrl_mode_trigger, sizeof(slv4_ctrl_mode_trigger), 1000 / portTICK_PERIOD_MS);

    if (ret == ESP_OK) {
        vTaskDelay(200 / portTICK_PERIOD_MS); // Allow magnetometer mode switch
        ESP_LOGI(TAG, "Step 16 - Magnetometer set to Continuous Measurement Mode 1 (~10 Hz).");

    // Step 17 Optional Readback Verification (Still BANK 3)
        uint8_t slv4_addr_readback_data[] = {ICM20948_I2C_SLV4_ADDR, ICM20948_MAG_ADDRESS | 0x80}; // Read flag
        uint8_t slv4_reg_readback_data[] = {ICM20948_I2C_SLV4_REG, AK09916_CNTL2_REG};
        uint8_t slv4_ctrl_readback_trigger[] = {ICM20948_I2C_SLV4_CTRL, 0x80}; // Enable transaction

        ret = i2c_master_transmit(dev_handle, slv4_addr_readback_data, sizeof(slv4_addr_readback_data), 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) ret = i2c_master_transmit(dev_handle, slv4_reg_readback_data, sizeof(slv4_reg_readback_data), 1000 / portTICK_PERIOD_MS);
        if (ret == ESP_OK) ret = i2c_master_transmit(dev_handle, slv4_ctrl_readback_trigger, sizeof(slv4_ctrl_readback_trigger), 1000 / portTICK_PERIOD_MS);

        vTaskDelay(200 / portTICK_PERIOD_MS); // Wait for readback verification

        uint8_t cntl2_readback_val = 0;
        if (ret == ESP_OK) {
             uint8_t slv4_di_reg = ICM20948_I2C_SLV4_DI;
             ret = i2c_master_transmit_receive(dev_handle, &slv4_di_reg, 1, &cntl2_readback_val, 1, 1000 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Allow magnetometer mode switch

        if (ret == ESP_OK) {
            if (cntl2_readback_val == AK09916_MODE_CONT_MEAS1) {
                ESP_LOGI(TAG, "Step 17 - Magnetometer CNTL2 register readback OK: 0x02", cntl2_readback_val);
            } else {
                ESP_LOGW(TAG, "Step 17 - Magnetometer CNTL2 readback mismatch! Expected 0x%02X, got 0x%02X", AK09916_MODE_CONT_MEAS1, cntl2_readback_val);
                mag_init_ok = false;
            }
        } else {
             ESP_LOGE(TAG, "Step 17 - Failed to readback Magnetometer CNTL2. Error: %s", esp_err_to_name(ret));
             mag_init_ok = false;
        }
         // --- End Verify ---
    } else {
        ESP_LOGE(TAG, "Step 17 - Failed to set Magnetometer measurement mode. Error: %s", esp_err_to_name(ret));
        mag_init_ok = false;
    }

    g_mag_ready = mag_init_ok;
    if (!g_mag_ready) {
        ESP_LOGW(TAG, "Magnetometer path not ready; mag-dependent commands should fail fast.");
    }

    ESP_LOGI(TAG, "ICM20948 Sensor initialization is complete.\n");
}
//Called from app_main
esp_err_t icm20948_read_sensor_data(i2c_master_dev_handle_t dev_handle,icm20948_data_t *sensor_data) {
    // Ensure BANK 0 before reading Accel/Gyro data
    if (icm20948_set_bank(dev_handle, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set bank 0 before reading sensor data.");
        // Zero out data on error
        sensor_data->accel_x = sensor_data->accel_y = sensor_data->accel_z = 0.0f;
        sensor_data->gyro_x  = sensor_data->gyro_y  = sensor_data->gyro_z  = 0.0f;
        return ESP_FAIL;
    }

    uint8_t reg_addr = ICM20948_ACCEL_XOUT_H; // Start address for Accel/Gyro block
    uint8_t raw_data[12];  // 6 bytes accel + 6 bytes gyro

    esp_err_t ret = i2c_master_transmit(dev_handle, &reg_addr, 1, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit Accel/Gyro start address. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_master_receive(dev_handle, raw_data, sizeof(raw_data), I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to retrieve Accel/Gyro data. Error: %s", esp_err_to_name(ret));
        return ret;
    }

    // Assuming default FSR: Accel �2g, Gyro �250 dps
    sensor_data->accel_x = ((int16_t)((raw_data[0] << 8) | raw_data[1])) / 16384.0f;
    sensor_data->accel_y = ((int16_t)((raw_data[2] << 8) | raw_data[3])) / 16384.0f;
    sensor_data->accel_z = ((int16_t)((raw_data[4] << 8) | raw_data[5])) / 16384.0f;

    sensor_data->gyro_x  = ((int16_t)((raw_data[6] << 8) | raw_data[7])) / 131.0f;
    sensor_data->gyro_y  = ((int16_t)((raw_data[8] << 8) | raw_data[9])) / 131.0f;
    sensor_data->gyro_z  = ((int16_t)((raw_data[10] << 8) | raw_data[11])) / 131.0f;

    // Success
    return ESP_OK;
}


void icm20948_calibrate_gyro_bias(i2c_master_dev_handle_t dev_handle, int samples) {
    ESP_LOGI(TAG, "\n\nGyro 2D calibration will start platform should be perfecly still!.\n"
                     "Data collection will last %.1f seconds...",
             (float)(GYRO_NUM_CALIBRATION_SAMPLES * GYRO_CALIBRATION_DELAY_MS) / 1000.0f);
    vTaskDelay(pdMS_TO_TICKS(5000));

    icm20948_data_t temp_data = {0};
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; i++) {
        esp_err_t ret = icm20948_read_sensor_data(dev_handle, &temp_data);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Gyro calibration sample %d failed, skipping", i);
            // Optionally zero out temp_data or just continue 
            continue; 
        }
        
        sum_x += temp_data.gyro_x;
        sum_y += temp_data.gyro_y;
        sum_z += temp_data.gyro_z;
        vTaskDelay(pdMS_TO_TICKS(GYRO_CALIBRATION_DELAY_MS));
    }

    // Update global bias
    g_gyro_bias.gyro_x = sum_x / samples;
    g_gyro_bias.gyro_y = sum_y / samples;
    g_gyro_bias.gyro_z = sum_z / samples;

    ESP_LOGI(TAG, "Gyro bias calibrated: X=%.4f Y=%.4f Z=%.4f",
             g_gyro_bias.gyro_x, g_gyro_bias.gyro_y, g_gyro_bias.gyro_z);

    // Store to NVS
    gyro_calibration_t cal = {
        .gyro_bias = {g_gyro_bias.gyro_x, g_gyro_bias.gyro_y, g_gyro_bias.gyro_z},
        .version = 1
    };

    esp_err_t err = store_gyro_calibration_to_nvs(&cal);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Gyro calibration saved to NVS.");
    } else {
        ESP_LOGW(TAG, "Failed to store gyro calibration to NVS: %s", esp_err_to_name(err));
    }

}

esp_err_t store_gyro_calibration_to_nvs(const gyro_calibration_t *cal) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("gyro_cal", NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, "cal_data", cal, sizeof(gyro_calibration_t));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

esp_err_t load_gyro_calibration_from_nvs(gyro_calibration_t *cal) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("gyro_cal", NVS_READONLY, &handle);
    if (err != ESP_OK) return err;

    size_t size = sizeof(gyro_calibration_t);
    err = nvs_get_blob(handle, "cal_data", cal, &size);
    nvs_close(handle);
    return err;
}

//Called from app_main: Reads magnetometer data from EXT_SLV_SENS_DATA_00 (0x3B in Bank 0)
// This function assumes you are in BANK 0 before calling it.
esp_err_t  icm20948_read_mag_data(i2c_master_dev_handle_t dev_handle, icm20948_mag_data_t *mag_data) {
    uint8_t reg_addr = ICM20948_EXT_SLV_SENS_DATA_00; // Start address for the 9 bytes of mag data
    uint8_t raw_data[AK09916_DATA_BYTES_TO_READ]; // Buffer for ST1, HXL..HZH, ST2
    esp_err_t ret;
    uint8_t st1 = 0; // Status 1 register value from magnetometer
    int retry_count = 0;
    const int max_retries = 100;
    const int delay_ms = 1;
    //const int delay_micro_sec = 1000;

    // Switch to BANK_0 ---
    ret = icm20948_set_bank(dev_handle, 0);
    if (ret != ESP_OK) {
        return ret; // Return the error code from icm20948_set_bank
    }

    // Retry loop to check for data readiness (DRDY bit in ST1)
    while (retry_count < max_retries) {
        // Read only the ST1 register first (EXT_SLV_SENS_DATA_00)
        ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &st1, 1, 1000 / portTICK_PERIOD_MS);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read ST1 byte for mag data readiness check. Error: %s", esp_err_to_name(ret));
            return ret; // Propagate underlying I2C error
        }

        ESP_LOGD(TAG, "Retry %d: Read ST1 = 0x%02X", retry_count, st1); // Log the value read

        // Check if DRDY bit (bit 0) in ST1 is set
        if (st1 & 0x01) {
            //ESP_LOGI(TAG, "Mag data ready (ST1=0x%02X) after %d retries.", st1, retry_count);
            break; // Data is ready, exit loop
        } else {
            //ESP_LOGD(TAG, "Mag data not ready (ST1=0x%02X) on retry %d. Retrying...", st1, retry_count); // Original log, now redundant
            retry_count++;
            vTaskDelay(pdMS_TO_TICKS(delay_ms)); // Wait before next retry
            //esp_rom_delay_us(delay_micro_sec);  // micro second delay
        }
    }

    // If data is still not ready after max retries, log a warning and return
    if (!(st1 & 0x01)) {
        ESP_LOGW(TAG, "Mag data not ready after max retries (%d). Skipping processing.", max_retries);
        return ESP_ERR_TIMEOUT; // Data-ready timeout
    }

    // --- Data should be ready now, read the full 9 bytes ---
    // The ICM master should have already read the 9 bytes into EXT_SLV_SENS_DATA_00 to _08
    // We just need to read them from the ICM registers.
    ret = i2c_master_transmit(dev_handle, &reg_addr, 1, 1000 / portTICK_PERIOD_MS); // Set read address to EXT_SLV_SENS_DATA_00
    if (ret == ESP_OK) {
        ret = i2c_master_receive(dev_handle, raw_data, sizeof(raw_data), 1000 / portTICK_PERIOD_MS); // Read all 9 bytes
        if (ret == ESP_OK) {
            // Process the raw_data bytes

            // Check ST2 register (raw_data[8] = EXT_SLV_SENS_DATA_08) for overflow
            uint8_t st2 = raw_data[8];
            if (st2 & 0x08) { // Check HOFL bit (bit 3)
                ESP_LOGW(TAG, "Magnetometer data overflow (ST2=0x%02X)!", st2);
                // Data might be invalid, maybe return or zero out data?
                // For now, we'll proceed but log the warning.
            }
            // Check ST1 register (raw_data[0] = EXT_SLV_SENS_DATA_0) for overrun
            if (st1 & 0x02) { // Check DOR bit (bit 1) - This might be expected if reads are slow
                 //ESP_LOGW(TAG, "Magnetometer data overrun (ST2=0x%02X)!", st2);
             }

            // Magnetometer data is 16-bit signed, Little Endian
            int16_t raw_mag_x = (int16_t)(((uint16_t)raw_data[2] << 8) | raw_data[1]); // HXH, HXL
            int16_t raw_mag_y = (int16_t)(((uint16_t)raw_data[4] << 8) | raw_data[3]); // HYH, HYL
            int16_t raw_mag_z = (int16_t)(((uint16_t)raw_data[6] << 8) | raw_data[5]); // HZH, HZL

            // Scale the data (Sensitivity is ~0.15 uT/LSB)
            mag_data->mag_x = (float)raw_mag_x * AK09916_SENSITIVITY;
            mag_data->mag_y = (float)raw_mag_y * AK09916_SENSITIVITY;
            mag_data->mag_z = (float)raw_mag_z * AK09916_SENSITIVITY;

            
            

        } else {
            ESP_LOGE(TAG, "Failed to retrieve full magnetometer data from EXT_SLV_SENS_DATA. Error: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
         ESP_LOGE(TAG, "Failed to transmit EXT_SLV_SENS_DATA address for full read. Error: %s", esp_err_to_name(ret));
         return ret;
    }
    return ESP_OK; // Indicate success
}

esp_err_t collect_mag_calibration_data(i2c_master_dev_handle_t dev_handle,
                                       raw_mag_sample_t mag_samples[]) {

    ESP_LOGI(TAG, "Collecting magnetometer samples...");

    vTaskDelay(pdMS_TO_TICKS(2000));

    icm20948_mag_data_t current_mag_data; 
    int64_t start_time_us = esp_timer_get_time();
    int valid_sample_count = 0;
    int total_attempt_count = 0;
    int read_error_count = 0;
    const int max_total_attempts = MAG_NUM_CALIBRATION_SAMPLES * 20;
    const int64_t max_collection_time_us = 120LL * 1000000LL; // 120 seconds

    while (valid_sample_count < MAG_NUM_CALIBRATION_SAMPLES) {
        total_attempt_count++;
        int64_t elapsed_time_us = esp_timer_get_time() - start_time_us;

        if (total_attempt_count > max_total_attempts || elapsed_time_us > max_collection_time_us) {
            ESP_LOGE(TAG,
                     "Mag calibration sample collection timed out: valid=%d/%d attempts=%d read_errors=%d elapsed=%.1fs",
                     valid_sample_count, MAG_NUM_CALIBRATION_SAMPLES, total_attempt_count, read_error_count,
                     (float)elapsed_time_us / 1000000.0f);
            return ESP_ERR_TIMEOUT;
        }

        esp_err_t read_ret = icm20948_read_mag_data(dev_handle, &current_mag_data);
        if (read_ret != ESP_OK) {
            read_error_count++;
            if ((read_error_count % 10) == 0 || read_error_count <= 3) {
                ESP_LOGW(TAG,
                         "Mag calibration read failed (%d errors, attempt %d): %s",
                         read_error_count, total_attempt_count, esp_err_to_name(read_ret));
            }
            vTaskDelay(pdMS_TO_TICKS(MAG_CALIBRATION_DELAY_MS));
            continue;
        }

        float mx = current_mag_data.mag_x;
        float my = current_mag_data.mag_y;
        float mz = current_mag_data.mag_z;

        // --- Outlier rejection ---
        if (isnan(mx) || isnan(my) || isnan(mz) ||
            isinf(mx) || isinf(my) || isinf(mz)) {
            ESP_LOGW(TAG, "Sample %d: NaN/Inf detected, skipping", valid_sample_count);
            continue;
        }

        if (fabs(mx) > 100.0f || fabs(my) > 100.0f || fabs(mz) > 100.0f) {
            ESP_LOGW(TAG, "Sample %d: Extreme value detected (%.2f, %.2f, %.2f), skipping",
                     valid_sample_count, mx, my, mz);
            continue;
        }

        float field_strength = sqrtf(mx*mx + my*my + mz*mz);
        if (field_strength < 10.0f || field_strength > 100.0f) {
            ESP_LOGW(TAG, "Sample %d: Abnormal field strength %.2f �T, skipping",
                     valid_sample_count, field_strength);
            continue;
        }
        // --- End outlier rejection ---

        mag_samples[valid_sample_count].mx = mx;
        mag_samples[valid_sample_count].my = my;
        mag_samples[valid_sample_count].mz = mz;
        valid_sample_count++;

        ESP_LOGI(TAG, "Sample %d: mx=%.3f, my=%.3f, mz=%.3f", 
            valid_sample_count, mx, my, mz);

        if ((valid_sample_count % (MAG_NUM_CALIBRATION_SAMPLES / 4) == 0) ||
            (valid_sample_count == MAG_NUM_CALIBRATION_SAMPLES)) {
            float elapsed_sec = (float)elapsed_time_us / 1000000.0f;
            ESP_LOGI(TAG, "Collected %d of %d valid samples (%.1f%%) in %.1f seconds...",
                     valid_sample_count, MAG_NUM_CALIBRATION_SAMPLES,
                     (float)valid_sample_count * 100.0f / MAG_NUM_CALIBRATION_SAMPLES,
                     elapsed_sec);
        }

        vTaskDelay(pdMS_TO_TICKS(MAG_CALIBRATION_DELAY_MS));
    }

    ESP_LOGI(TAG, "Magnetometer sample collection complete.");
    vTaskDelay(pdMS_TO_TICKS(2000));
    return ESP_OK;
}


/**
 * A 2D hard-iron magnetometer calibration function that analyzes collected data.
 *
 * @param[in] dev_handle I2C device handle for ICM20948.
 * @param[in] mag_samples An array of raw_mag_sample_t containing the collected data.
 * @param[in] num_samples The number of samples in the mag_samples array.
 * @param[out] calibration_results Pointer to a mag_calibration_t struct to store the results.
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t icm20948_calibrate_mag_2d(i2c_master_dev_handle_t dev_handle, raw_mag_sample_t mag_samples[], int num_samples, mag_calibration_t *calibration_results)
{
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;

    if (num_samples == 0 || !mag_samples) {
        ESP_LOGE("CALIB_2D", "No samples provided for calibration.");
        return ESP_FAIL;
    }

    // Initialize min/max with the first sample
    if (num_samples > 0) {
        min_x = max_x = mag_samples[0].mx;
        min_y = max_y = mag_samples[0].my;
    }

    for (int i = 1; i < num_samples; i++) {
        if (mag_samples[i].mx < min_x) min_x = mag_samples[i].mx;
        if (mag_samples[i].mx > max_x) max_x = mag_samples[i].mx;
        if (mag_samples[i].my < min_y) min_y = mag_samples[i].my;
        if (mag_samples[i].my > max_y) max_y = mag_samples[i].my;
    }

    ESP_LOGI(TAG, "  min x: %.4f uT", min_x);
    ESP_LOGI(TAG, "  max x: %.4f uT", max_x);
    ESP_LOGI(TAG, "  min y: %.4f uT", min_y);
    ESP_LOGI(TAG, "  max y: %.4f uT", max_y);
    
    // Calculate the hard-iron bias by averaging the min and max values
    float hard_iron_bias_x = (min_x + max_x) / 2.0f;
    float hard_iron_bias_y = (min_y + max_y) / 2.0f;

    // Calculate the compensated radius
    float radius = ((max_x - min_x) + (max_y - min_y)) / 4.0f;

    if (calibration_results) {
        calibration_results->hard_iron_bias[0] = hard_iron_bias_x;
        calibration_results->hard_iron_bias[1] = hard_iron_bias_y;
        
        // Calculate average Z bias instead of setting to 0
        float sum_z = 0.0f;
        int valid_z_samples = 0;
        
        for (int i = 0; i < num_samples; i++) {
            // Apply outlier rejection to Z samples
            if (!isnan(mag_samples[i].mz) && !isinf(mag_samples[i].mz) && 
                fabs(mag_samples[i].mz) <= 100.0f) {
                sum_z += mag_samples[i].mz;
                valid_z_samples++;
            }
        }
        
        if (valid_z_samples > 0) {
            calibration_results->hard_iron_bias[2] = sum_z / valid_z_samples;
            ESP_LOGI(TAG, "  Calculated Z bias from %d valid samples", valid_z_samples);
        } else {
            calibration_results->hard_iron_bias[2] = 0.0f;
            ESP_LOGW(TAG, "  No valid Z samples found, using 0.0f");
        }

        // Soft-iron matrix for 2D circle fit is identity
        memset(calibration_results->soft_iron_matrix, 0, sizeof(calibration_results->soft_iron_matrix));
        calibration_results->soft_iron_matrix[0][0] = 1.0f;
        calibration_results->soft_iron_matrix[1][1] = 1.0f;
        calibration_results->soft_iron_matrix[2][2] = 1.0f;

        // Set calibration format version
        calibration_results->version = 1;
    }

    ESP_LOGI(TAG, "2D Calibration Results:");
    ESP_LOGI(TAG, "  Hard-Iron Bias X: %.4f uT", hard_iron_bias_x);
    ESP_LOGI(TAG, "  Hard-Iron Bias Y: %.4f uT", hard_iron_bias_y);
    ESP_LOGI(TAG, "  Hard-Iron Bias Z: %.4f uT", calibration_results->hard_iron_bias[2]);
    ESP_LOGI(TAG, "  Compensated Radius: %.4f uT", radius);

    return ESP_OK;
}

esp_err_t icm20948_calibrate_mag_3d(i2c_master_dev_handle_t dev_handle,
                                    raw_mag_sample_t mag_samples[],
                                    int num_samples,
                                    mag_calibration_t *calibration_results)
{
    if (num_samples < 50 || !mag_samples) {
        ESP_LOGE("CALIB_3D", "Not enough samples for 3D calibration.");
        return ESP_FAIL;
    }

    // --- Step 1: Use precomputed calibration results (from Python) ---
    float bias[3] = { -10.88f,  23.10f, -48.98f };
    float soft_iron[3][3] = {
        { 1.008f, -0.015f,  0.033f },
        { -0.015f,  0.963f,  0.002f },
        { 0.033f,   0.002f,  1.024f }
    };

    // --- Step 2: Store results ---
    calibration_results->hard_iron_bias[0] = bias[0];
    calibration_results->hard_iron_bias[1] = bias[1];
    calibration_results->hard_iron_bias[2] = bias[2];
    memcpy(calibration_results->soft_iron_matrix, soft_iron, sizeof(soft_iron));

    calibration_results->version = 3;

    // --- Step 3: Log results ---
    ESP_LOGI(TAG, "3D Calibration Results:");
    ESP_LOGI(TAG, "  Hard-Iron Bias: X=%.3f, Y=%.3f, Z=%.3f",
             calibration_results->hard_iron_bias[0],
             calibration_results->hard_iron_bias[1],
             calibration_results->hard_iron_bias[2]);
    ESP_LOGI(TAG, "  Soft-Iron Matrix:");
    ESP_LOGI(TAG, "[%.3f %.3f %.3f]",
             calibration_results->soft_iron_matrix[0][0],
             calibration_results->soft_iron_matrix[0][1],
             calibration_results->soft_iron_matrix[0][2]);
    ESP_LOGI(TAG, "[%.3f %.3f %.3f]",
             calibration_results->soft_iron_matrix[1][0],
             calibration_results->soft_iron_matrix[1][1],
             calibration_results->soft_iron_matrix[1][2]);
    ESP_LOGI(TAG, "[%.3f %.3f %.3f]",
             calibration_results->soft_iron_matrix[2][0],
             calibration_results->soft_iron_matrix[2][1],
             calibration_results->soft_iron_matrix[2][2]);

    return ESP_OK;
}

esp_err_t store_mag_calibration_to_nvs(const mag_calibration_t *cal) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("mag_cal", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(nvs_handle, "cal_data", cal, sizeof(mag_calibration_t));
    if (err != ESP_OK) {
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    return err;
}

esp_err_t load_mag_calibration_from_nvs(mag_calibration_t *cal) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("mag_cal", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(mag_calibration_t);
    err = nvs_get_blob(nvs_handle, "cal_data", cal, &required_size);
    nvs_close(nvs_handle);

    return err;
}

esp_err_t icm20948_read_mag_calibrated(i2c_master_dev_handle_t dev_handle,
                                       const mag_calibration_t *bias,
                                       icm20948_mag_data_t *mag_cal) {
    icm20948_mag_data_t raw;
    esp_err_t ret = icm20948_read_mag_data(dev_handle, &raw);
    if (ret != ESP_OK) { 
        mag_cal->mag_x = mag_cal->mag_y = mag_cal->mag_z = 0.0f; 
        return ret; 
    }

    // Debug log raw values before calibration 
    
    //ESP_LOGI(TAG, "Raw magnetometer: x=%.2f, y=%.2f, z=%.2f", 
    //    raw.mag_x, raw.mag_y, raw.mag_z);

    // Debug log hard-iron bias values 
    //ESP_LOGI(TAG, "Hard-iron bias: bx=%.2f, by=%.2f, bz=%.2f", 
    //    bias->hard_iron_bias[0], 
    //    bias->hard_iron_bias[1], 
    //    bias->hard_iron_bias[2]);


    // Apply hard-iron bias
    float mx = raw.mag_x - bias->hard_iron_bias[0];
    float my = raw.mag_y - bias->hard_iron_bias[1];
    float mz = raw.mag_z - bias->hard_iron_bias[2];

    //ESP_LOGI(TAG, "After bias subtraction: x=%.2f, y=%.2f, z=%.2f", mx, my, mz);

    // Apply soft-iron correction
    mag_cal->mag_x = bias->soft_iron_matrix[0][0]*mx +
                     bias->soft_iron_matrix[0][1]*my +
                     bias->soft_iron_matrix[0][2]*mz;
    mag_cal->mag_y = bias->soft_iron_matrix[1][0]*mx +
                     bias->soft_iron_matrix[1][1]*my +
                     bias->soft_iron_matrix[1][2]*mz;
    mag_cal->mag_z = bias->soft_iron_matrix[2][0]*mx +
                     bias->soft_iron_matrix[2][1]*my +
                     bias->soft_iron_matrix[2][2]*mz;

    //ESP_LOGI(TAG, "After soft-iron correction: x=%.2f, y=%.2f, z=%.2f", 
    //    mag_cal->mag_x, mag_cal->mag_y, mag_cal->mag_z);

    return ESP_OK;
}

esp_err_t icm20948_read_gyro_calibrated(i2c_master_dev_handle_t dev_handle,
                                        const icm20948_data_t *gyro_bias,
                                        icm20948_data_t *gyro_cal) {
    icm20948_data_t raw;
    esp_err_t ret = icm20948_read_sensor_data(dev_handle, &raw);
    if (ret != ESP_OK) { 
        memset(gyro_cal, 0, sizeof(*gyro_cal)); 
        return ret; 
    }

    gyro_cal->gyro_x = raw.gyro_x - gyro_bias->gyro_x;
    gyro_cal->gyro_y = raw.gyro_y - gyro_bias->gyro_y;
    gyro_cal->gyro_z = raw.gyro_z - gyro_bias->gyro_z;

    // Pass through accel unchanged for now
    gyro_cal->accel_x = raw.accel_x;
    gyro_cal->accel_y = raw.accel_y;
    gyro_cal->accel_z = raw.accel_z;

    return ESP_OK;
}

float icm20948_compute_heading(const icm20948_mag_data_t *mag) {
    float heading = atan2f(mag->mag_x, mag->mag_y) * (180.0f / M_PI);
    if (heading < 0) heading += 360.0f; // normalize to 0-360
    return heading;
}
