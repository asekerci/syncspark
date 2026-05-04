# get_esp32_chip_info: For Testing ESP32 Boards  

## ESP32-CAM Board

Configuration details (including the partition table) are in 
the main [README.md](../README.md) file of the **SyncSpark** project. 

The steps for flashing and running `get_esp32_chip_info` are as follows. We assume that the ESP-IDF toolchain is installed in the `$HOME/esp` directory, the current version is v5.5, and the GitHub SyncSpark repository is cloned into `$HOME/projects/syncspark`.

To generate the binary and flash it into the chip:

1. `source $HOME/esp/v5.5/esp-idf/export.sh` to set up the necessary environment variables
2. `cd $HOME/projects/syncspark/get_esp32_chip_info`
3. `idf.py reconfigure` to generate `sdkconfig` from `sdkconfig.defaults`
4. `idf.py build` to produce the binary
5. Connect the ESP32-CAM to your computer via a USB cable. Press the IO0 button and, while keeping it pressed, press the ESP32-CAM reset button. Release the reset button first, then release the IO0 button. The chip is now ready to be flashed.
6. `idf.py flash monitor` to flash the firmware into the factory partition
7. Press the reset button at the back of the ESP32-CAM board to run the freshly flashed firmware

## Lilygo T-Display Board

Joe bought these boards for us:
- ESP32D0WD Q6 chip
- 16 MB flash memory
- 448 KB ROM 
- 520 KB on chip SRAM (data + instructions)
- 8 KB SRAM in the RTC
- The display uses the ST7789 driver that supports RGB565 color format.

The TS0636G-T-Display is commonly recognized as the LilyGO TTGO
T-Display ESP32 development board. It is powered by the ESP32-D0WD-Q6
microcontroller with a dual-core 240 MHz processor, 520 KB of SRAM,
and 4 MB of flash memory. This board features an integrated 1.14-inch
IPS display (ST7789 driver) with a resolution of 135x240 pixels, which
connects via the SPI interface.

These boards do not have PSRAMs unlike the esp32-cams I have. 

For the Lilygo T-Display unit, run `idf.py menuconfig`, and do the following:
1. "serial flasher config": Increase flash size to 16 MB
2. "Component config --> ESP System Settings": Increase the clock speed to 240 MHz
3. "Component config --> ESP PSRAM": Enable support for PSRAM, and tick "Ignore PSRAM when not found". 

The Lilygo T-Display unit does not need to be placed into bootloader
mode (unlike the esp32-cams: In order to place them into bootloader
mode, you need to press and hold IO0 button, while holding it, press
and release the reset button at the back of esp32-cam. Then release
the IO0 button).

Just issue `idf.py flash` command for the Lilygo T-Display units. 



