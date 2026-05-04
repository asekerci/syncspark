/* SPDX-License-Identifier: GPL-3.0-or-later */

/*
 * bmp388.c - BMP388 barometric pressure sensor driver implementation
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

/*# Summary of BMP388.c: Reading, Calibration, and Compensation

1. **Sensor Initialization:**
   - The `bmp388_initialize()` function begins by sending a soft reset command to the
     sensor via I²C to restore default settings.
   - After short delays for stability, it sends I²C configuration commands to set:
     - The power mode (Normal Mode, enabling both temperature and pressure measurements),
     - Oversampling parameters (e.g., Pressure ×8, Temperature ×1),
     - Output data rate (ODR), and
     - IIR filter coefficients.
   - Once configuration is complete, the initialization function calls
     `bmp388_read_coefficients()` to retrieve the sensor's 
     calibration data.

2. **Calibration Data Acquisition and Parsing:**
   - The `bmp388_read_coefficients()` function reads 21 consecutive bytes from the BMP388’s
     calibration register.
   - These 21 bytes contain factory-calibrated values, which are combined using
     bitwise operations and then scaled using specific 
     divisors (e.g., 0.00390625, 1073741824.0, 281474976710656.0, etc.).
   - Temperature calibration coefficients (par_t1, par_t2, par_t3) and pressure
     calibration coefficients (par_p1 through par_p11) are stored in a calibration
     structure (`bmp388_calib_data_t`) as double‑precision values, ensuring high
     accuracy in subsequent calculations.

3. **Temperature Compensation:**
   - **Reading Raw Temperature:**
     - The raw temperature value is obtained from 3 bytes (forming a 24-bit value).
   - **Compensation Calculation:**
     - The function `bmp388_compensate_temperature_double()` computes:
       - `partial_data1 = (raw temperature - par_t1)`
       - `partial_data2 = partial_data1 * par_t2`
       - It then calculates a linearized temperature (`t_lin`) as:
         - `t_lin = partial_data2 + (partial_data1^2 * par_t3)`
       - This `t_lin` serves both as the compensated temperature output and as a
         critical input for pressure compensation.
   - A float wrapper (`bmp388_compensate_temperature()`) converts the result to float if needed.

4. **Pressure Compensation:**
   - **Reading Raw Pressure:**
     - The raw pressure is similarly collected from 3 bytes (forming a 24-bit value).
   - **Compensation Calculation:**
     - The function `bmp388_compensate_pressure_double()` uses the previously computed linearized temperature (`t_lin`) along 
     with the raw pressure reading.
     - Several intermediate terms are calculated:
       - **Temperature-Dependent Offset:**  
         Terms from the calibration coefficients (par_p5, par_p6, par_p7, par_p8) multiplied by `t_lin`, `t_lin²`, and `t_lin³`.
       - **Linear Pressure Scaling:**  
         The raw pressure is multiplied by a combination of coefficients (par_p1, par_p2, par_p3, par_p4) which have also been 
         corrected for temperature.
       - **High-Order Corrections:**  
         Terms involving the square and cube of the raw pressure are multiplied by calibration factors (par_p9, par_p10, par_p11).
     - The final compensated pressure is the sum of all these computed components.
   - Like temperature, a float wrapper (`bmp388_compensate_pressure()`) is provided if a float result is desired.

5. **Logging the Results:**
   - After compensation is complete, the code logs:
     - The raw sensor values,
     - The computed compensated temperature and pressure,
   using ESP logging functions. This diagnostic output helps verify sensor readings and ensure that calibration is applied correctly.

In summary, the BMP388 driver:
- Initializes and configures the sensor,
- Reads calibration data and converts it into high-precision values,
- Applies a two-stage compensation—first for temperature (yielding t_lin) and then for pressure using t_lin,
- Logs all key values for diagnostic purposes.

This structured process ensures the final sensor readings accurately reflect the ambient temperature and pressure, compensating for 
inherent sensor non-linearities.
*/

#include "bmp388.h"

static bmp388_calib_data_t calib_data;
static const char *TAG = "BMP388";

void initialize_bmp388(i2c_master_dev_handle_t bmp388_dev_handle, int i2c_master_timeout_ms) 
{
    esp_err_t ret;

    // Step 1: Perform a soft reset
    // All user configuration settings are overwritten with their default state
    uint8_t soft_reset_cmd[] = {BMP388_CMD_REG, 0xB6}; 

    ret = i2c_master_transmit(bmp388_dev_handle, soft_reset_cmd, sizeof(soft_reset_cmd), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 Soft Reset triggered.");
    } else {
        ESP_LOGE(TAG, "BMP388 Soft Reset failed. Error: %s", esp_err_to_name(ret));
        return;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Stability delay
    // Wait before the next iteration - probably we don't need this in the final code 
    // vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for 1 seconds

    // Step 2: Set power mode (normal Mmode)
    uint8_t power_mode_data[] = {
        BMP388_PWR_CTRL_REG,  // Power control register
        0x33   // Set Normal Mode (bits [5:4]), enable pressure + temperature sensors (bits [1:0])
    };
    ret = i2c_master_transmit(bmp388_dev_handle, power_mode_data, sizeof(power_mode_data), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 Power Mode set to Normal.");
    } else {
        ESP_LOGE(TAG, "Failed to set BMP388 Power Mode. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Step 3: Set oversampling for pressure and temperature
    uint8_t osr_data[] = {
        BMP388_OSR_REG,  // Oversampling register
        0x03   // Pressure: x8 (bits [3:1]), Temperature: x1 (bit 0)
    };
    ret = i2c_master_transmit(bmp388_dev_handle, osr_data, sizeof(osr_data), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 Oversampling set (Pressure x8, Temperature x1).");
    } else {
        ESP_LOGE(TAG, "Failed to set BMP388 Oversampling. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Step 4: Set Output Data Rate (ODR)
    uint8_t odr_data[] = {
        BMP388_ODR_REG,  // ODR Output Data Rate Register
        //0x02   // Set ODR to 50 Hz (~20 ms interval)
        0x07   // Set ODR to 25/16 Hz (~640 ms interval)
    };
    ret = i2c_master_transmit(bmp388_dev_handle, odr_data, sizeof(odr_data), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 Output Data Rate set to 25/16 Hz.");
    } else {
        ESP_LOGE(TAG, "Failed to set BMP388 Output Data Rate. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Step 5: Configure IIR Filter Coefficients
    uint8_t iir_filter_data[] = {
        BMP388_CONFIG_REG,  // IIR filter Coefficients Register
        0x02   // IIR Filter Coefficient = 3
    };
    ret = i2c_master_transmit(bmp388_dev_handle, iir_filter_data, sizeof(iir_filter_data), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BMP388 IIR Filter Coefficients set to 3.");
    } else {
        ESP_LOGE(TAG, "Failed to set BMP388 IIR Filter Coefficients. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Step 6: Read Calibration Coefficients
    bmp388_read_coefficients(bmp388_dev_handle, &calib_data, i2c_master_timeout_ms);  // Retrieve and parse calibration coefficients

    // Final Step: Stability Delay
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Allow sensor to stabilize
    ESP_LOGI(TAG, "BMP388 initialization complete.");
    // Wait before the next iteration
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait for 1 seconds
} // initialize_bmp388()

//****************************************************************************************************
/* Updated calibration reading function using DFRobot divisors */
void bmp388_read_coefficients(i2c_master_dev_handle_t dev_handle, 
                             bmp388_calib_data_t *calib_data, int i2c_master_timeout_ms) {
    uint8_t calib_data_raw[21];
    uint8_t calib_addr = BMP388_CALIB_ADDR;
    esp_err_t ret;

    ESP_LOGI(TAG, "Reading calibration coefficients from BMP388.");
    ret = i2c_master_transmit(dev_handle, &calib_addr, sizeof(calib_addr), i2c_master_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send calibration address. Error: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_master_receive(dev_handle, calib_data_raw, sizeof(calib_data_raw), i2c_master_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to retrieve calibration coefficients. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Temperature calibration coefficients
    // par_t1: divide by 0.00390625 (i.e. multiply by 256)
    calib_data->par_t1 = ((double)(((uint16_t)calib_data_raw[1] << 8) | calib_data_raw[0])) / 0.00390625;
    // par_t2: divide by 2^30 (1073741824.0)
    calib_data->par_t2 = ((double)(((uint16_t)calib_data_raw[3] << 8) | calib_data_raw[2])) / 1073741824.0;
    // par_t3: divide by 2^48 (281474976710656.0)
    calib_data->par_t3 = ((double)((int8_t)calib_data_raw[4])) / 281474976710656.0;

    // Pressure calibration coefficients
    calib_data->par_p1  = (((double)((int16_t)(((uint16_t)calib_data_raw[6] << 8) | calib_data_raw[5]) - 16384)))/1048576.0;
    calib_data->par_p2  = (((double)((int16_t)(((uint16_t)calib_data_raw[8] << 8) | calib_data_raw[7]) - 16384)))/536870912.0;
    calib_data->par_p3  = ((double)((int8_t)calib_data_raw[9]))/4294967296.0;
    calib_data->par_p4  = ((double)((int8_t)calib_data_raw[10]))/137438953472.0;
    calib_data->par_p5  = ((double)(((uint16_t)calib_data_raw[12] << 8) | calib_data_raw[11]))/0.125;
    calib_data->par_p6  = ((double)(((uint16_t)calib_data_raw[14] << 8) | calib_data_raw[13]))/64.0;
    calib_data->par_p7  = ((double)((int8_t)calib_data_raw[15]))/256.0;
    calib_data->par_p8  = ((double)((int8_t)calib_data_raw[16]))/32768.0;
    calib_data->par_p9  = ((double)((int16_t)(((uint16_t)calib_data_raw[18] << 8) | calib_data_raw[17])))/281474976710656.0;
    calib_data->par_p10 = ((double)((int8_t)calib_data_raw[19]))/281474976710656.0;
    calib_data->par_p11 = ((double)((int8_t)calib_data_raw[20]))/36893488147419103232.0;

    ESP_LOGI(TAG, "Calibration Coefficients (Double):");
    ESP_LOGI(TAG, "par_t1: %.6f, par_t2: %.10f, par_t3: %.10f", calib_data->par_t1, calib_data->par_t2, calib_data->par_t3);
    ESP_LOGI(TAG, "par_p1: %.6f, par_p2: %.6f, par_p3: %.6f", calib_data->par_p1, calib_data->par_p2, calib_data->par_p3);
    ESP_LOGI(TAG, "par_p4: %.6f, par_p5: %.6f, par_p6: %.6f", calib_data->par_p4, calib_data->par_p5, calib_data->par_p6);
    ESP_LOGI(TAG, "par_p7: %.6f, par_p8: %.6f, par_p9: %.6f", calib_data->par_p7, calib_data->par_p8, calib_data->par_p9);
    ESP_LOGI(TAG, "par_p10: %.6f, par_p11: %.6f", calib_data->par_p10, calib_data->par_p11);
} // bmp388_read_coefficients()

//****************************************************************************************************
/* Use double-precision; then cast to float if needed */
double bmp388_compensate_temperature_double(uint32_t uncomp_temp, bmp388_calib_data_t *calib_data) 
{
    double partial_data1 = (double)uncomp_temp - calib_data->par_t1;
    double partial_data2 = partial_data1 * calib_data->par_t2;
    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1 * calib_data->par_t3);
    return calib_data->t_lin;
} // bmp388_compensate_temperature_double()

//****************************************************************************************************
double bmp388_compensate_pressure_double(uint32_t uncomp_press, bmp388_calib_data_t *calib_data) {
    double partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    double partial_data2 = calib_data->par_p7 * pow(calib_data->t_lin, 2);
    double partial_data3 = calib_data->par_p8 * pow(calib_data->t_lin, 3);
    double partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * pow(calib_data->t_lin, 2);
    partial_data3 = calib_data->par_p4 * pow(calib_data->t_lin, 3);
    double partial_out2 = ((double)uncomp_press) * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = pow((double)uncomp_press, 2);
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    double partial_data4 = partial_data3 + pow((double)uncomp_press, 3) * calib_data->par_p11;
    double comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
} // bmp388_compensate_pressure_double()

//****************************************************************************************************
/* Provide float wrappers if needed */
float bmp388_compensate_temperature(uint32_t uncomp_temp, bmp388_calib_data_t *calib_data) 
{
    return (float) bmp388_compensate_temperature_double(uncomp_temp, calib_data);
} // bmp388_compensate_temperature()

float bmp388_compensate_pressure(uint32_t uncomp_press, bmp388_calib_data_t *calib_data) 
{
    return (float) bmp388_compensate_pressure_double(uncomp_press, calib_data);
} // bmp388_compensate_pressure()

//****************************************************************************************************
void bmp388_read_data(i2c_master_dev_handle_t dev_handle, int i2c_master_timeout_ms) 
{
    union bmp388_raw_data_t {
        uint8_t raw[6];
        struct {
            uint32_t uncomp_press : 24;
            uint32_t uncomp_temp  : 24;
        } fields;
    } data;
    uint8_t data_addr = BMP388_DATA_ADDR;
    esp_err_t ret;

    ESP_LOGI(TAG, "Starting raw data read from BMP388.");
    ret = i2c_master_transmit(dev_handle, &data_addr, sizeof(data_addr), i2c_master_timeout_ms);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Address transmitted successfully. Reading raw data...");
        ret = i2c_master_receive(dev_handle, data.raw, sizeof(data.raw), i2c_master_timeout_ms);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Raw data successfully received.");

            uint32_t uncomp_press = (data.raw[2] << 16) | (data.raw[1] << 8) | data.raw[0];
            uint32_t uncomp_temp  = (data.raw[5] << 16) | (data.raw[4] << 8) | data.raw[3];

            ESP_LOGI(TAG, "Raw Pressure: %" PRIu32 ", Raw Temperature: %" PRIu32, uncomp_press, uncomp_temp);

            float compensated_temp = bmp388_compensate_temperature(uncomp_temp, &calib_data);
            float compensated_press = bmp388_compensate_pressure(uncomp_press, &calib_data);

            ESP_LOGI(TAG, "Compensated Temperature: %.2f °C", compensated_temp);
            ESP_LOGI(TAG, "Compensated Pressure: %.2f Pa", compensated_press);
        } else {
            ESP_LOGE(TAG, "Failed to retrieve raw data. Error: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to send data address. Error: %s", esp_err_to_name(ret));
    }
} // bmp388_read_data()

//****************************************************************************************************
void read_bmp388_chip_id(i2c_master_dev_handle_t bmp388_dev_handle, int i2c_master_timeout_ms) 
{
    uint8_t reg_addr = BMP388_CHIP_ID_REG;  // Register to read chip ID
    uint8_t chip_id = 0;  // Buffer to store the chip ID
    esp_err_t ret;

    // Write the chip ID register address to BMP388
    ret = i2c_master_transmit(bmp388_dev_handle, &reg_addr, sizeof(reg_addr), i2c_master_timeout_ms);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit register address to BMP388. Error: %s", esp_err_to_name(ret));
        return;
    }

    // Read the chip ID value from BMP388
    ret = i2c_master_receive(bmp388_dev_handle, &chip_id, sizeof(chip_id), i2c_master_timeout_ms);
    if (ret == ESP_OK && chip_id == BMP388_CHIP_ID) {
        ESP_LOGI(TAG, "BMP388 detected successfully. Chip ID: 0x%02X\n\n", chip_id);
    } else {
        ESP_LOGE(TAG, "Failed to detect BMP388 or invalid Chip ID. Error: %s", esp_err_to_name(ret));
    }
} // read_bmp388_chip_id()
