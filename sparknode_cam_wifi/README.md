# _sparknode_cam_wifi_

## ESP32-CAM
Clock speed: 240 MHz, 520 KB RAM, 8 MB PSRAM (SPIRAM), 4 MB flash memory

## About the Program

This program takes a picture every 20 seconds and sends it to a
destination over UDP.

My starting point was the example program provided in the Espressif
esp32-camera component. I removed the parts that are not related
esp32-cam. 

## ESP32-CAM and ESP-IDF Configuration

Configuration details  are in 
the main [README.md](../README.md) file of the **SyncSpark** project.

### Required Components

1. Run this command in the top directory of the project to add the `espressif/esp32-camera component`: 
   `idf.py add-dependency "espressif/esp32-camera^2.0.15"`
2. If you want to have more control over the camera and
   how it will be initialized, you can have your own `Kconfig` file. Copy the content from
   `https://github.com/espressif/esp32-camera/blob/master/Kconfig` into a local `Kconfig`.
   This file should reside in the `main` directory. Currently we use the default one residing in the esp32-camera component.

## Usage and Specific Configurations

### Wi-Fi Connection

Copy the file `wifi_credentials.h.template` to `wifi_credentials.h` (in the `main` directory) and then edit it to set the WiFi SSID and password to connect to your own Wi-Fi network (`wifi_credentials.h` will **not** be uploaded to the github repository). 

### Network Configuration 

We send two separate UDP datagrams to a remote host: One for images, 
and one for the copies of the log messages. 
See `network_config.h.template` for further details. Copy `network_config.h.template` to a file `network_config.h` and edit the remote host's details. 

### Receiving Images

In my case, the destination is my homelab Linux server `rama.local` (`192.168.1.201`), and 
the destination UDP port is `10000`.

On `rama`, I run the python program `rcv_frames_udp.py` to save 
the received image frames. The program is in the top project directory. It saves the received frames in the `frames` directory. In order to run it on `rama`, I first link my RPi5 home directory to `rama`'s `~/systems/sinope` directory by using the command 
`sshfs sinope.local: ~/systems/sinope`. Then, I run the command `rcv_frames_udp.py` on `rama` to listen to the messages sent by the robot (to-do: we need to create a more sophisticated program for receiving image frames from multiple robots simultaneously).

### Receiving Copies of Log messages

The robot sends the copies of log messages to UDP port 10001 of `rama`. Run the command `nc -u -l 10001` on a terminal to display them.  

## Espressif's Notes

- A recommended way is to follow the instructions on this [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).


