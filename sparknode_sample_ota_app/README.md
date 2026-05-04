# SparkNode Sample Over-The-Air (OTA) Application
## Overview

When the chip is programmed for the very first time, 
we flash `sparknode_ota_updater` in the `factory` partition by using a
USB cable.

 Then, whenever the chip is rebooted, or power-cycled, it 
 downloads a binary file and its corresponding MD5 checksum from the server specified in `network_config.h`. If the download is successful, the binary is written to the `ota_0` partition, the boot partition is set to `ota_0`, and the chip is restarted.

On subsequent boots, if the downloaded binary is properly configured 
the application changes the boot partition to `factory` to check the 
availability of updated firmware by running `sparknode_ota_updater` 
at every restart. 

This is a sample application that can be used as a starter for all
firmware sources for the SynchroSpark Project. 

## ESP32-CAM and ESP-IDF Configuration

Configuration details (including the partition table) are in 
the main [README.md](../README.md) file of the **SyncSpark** project.

## Software Configuration Before the First Compilation 

Configuration details (including the partition table) are in 
the main [README.md](../README.md) file of the **SyncSpark** project.

## Required ESP-IDF Components

- led_strip, run this command:
```bash
   idf.py add-dependency "espressif/led_strip^3.0.0"
```
