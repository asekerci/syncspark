# ICM20948 IMU Chip Calibration and Experiments
## Overview

- Supports the ICM20948 9-axis IMU sensor and BMP388 barometric pressure sensor via I2C.
- Performs sensor calibration and data acquisition.
- Implements Madgwick sensor fusion algorithm for orientation estimation.

## ESP32-CAM and ESP-IDF Configuration

Configuration details are in 
the main [README.md](../README.md) file of the **SyncSpark** project.

## Required Components

Run the following command  in the top directory of the project:

1. **espressif/led_strip**:
 `idf.py add-dependency "espressif/led_strip^3.0.0"`





