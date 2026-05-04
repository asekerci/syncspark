# MQTT Demo

## Overview

This ESP-IDF project is for testing the MQTT scheme, which enables remote 
control of SparkNodes via MQTT commands. 

The remote control commands include driving forward/reverse, turn left/right, stop
immediately, and reboot on command. We keep adding the new ones.    

The details are [here](../docs/software/mqtt.md).

## ESP32-CAM and ESP-IDF Configuration

Configuration details (including the partition tabl) are in the main
[README.md](../README.md) file of the **SyncSpark** project.

## Required Components

1.  espressif/led_strip:\
    `idf.py add-dependency "espressif/led_strip^3.0.0"`


