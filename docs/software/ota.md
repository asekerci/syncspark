#### OTA: One-Time Setup

Before generating any binaries, complete the following steps:

1. **Configure template files:** Locate
   [`wifi_credentials.h.template`](./components/syncspark_config/include/wifi_credentials.h.template)
   and
   [`network_config.h.template`](./components/syncspark_config/include/network_config.h.template)
   in the
   [`components/syncspark_config/include`](./components/syncspark_config/include/)
   directory, then follow these steps:
   1. **WiFi Connection**: Copy the contents of `wifi_credentials.h.template`
      to a new file named `wifi_credentials.h`[^3] and update the WiFi
      connection information as described in the file comments.
   2. **IP Address Configuration**: Copy the contents of
      `network_config.h.template` to a new file named `network_config.h`[^4]
      and update the IP addresses as described in the comments.
2. **Flash the OTA updater:** Flash `sparknode_ota_updater` directly over USB
   cable (**this will be placed in the _factory_ partition by default**), then
   disconnect the USB cable. Here are the **detailed flashing steps**
   (assuming the ESP-IDF toolchain is installed in `$HOME/esp`, version v5.5,
   and the SyncSpark repository is cloned to `$HOME/projects/syncspark`):
   1. Set up the environment: `source $HOME/esp/v5.5/esp-idf/export.sh`
   2. Navigate to the updater directory:
      `cd $HOME/projects/syncspark/sparknode_ota_updater`
   3. Configure the project: `idf.py reconfigure` (this command generates a
      `sdkconfig` from the `sdkconfig.defaults`).
   4. Build the binary: `idf.py build`
   5. Prepare the ESP32-CAM for flashing: Connect the ESP32-CAM to your
      computer via a USB cable. Press and hold the IO0 button, then press the
      ESP32-CAM reset button. Release the reset button first, then release the
      IO0 button. The chip is now ready for flashing.
   6. Flash and monitor: `idf.py flash monitor` (this command flashes the
      firmware to the factory partition).

The board is now ready for OTA updates. Disconnect the USB cable.

#### OTA: Typical Workflow

To generate a binary for an existing [SynchroSpark
program](#programs-for-testing-and-demonstrating-various-functionalities)[^5] in
the repository (for example, to run `sparknode_sample_ota_app` on a SparkNode
after cloning the SynchroSpark project sources to `$HOME/projects/syncspark`),
follow these steps:

1. **Activate the HTTP Server and Log Listener:**
   - Run the HTTP server on the target host (see the command in `network_config.h`)
   - Run [`log_listener.py`](./system/utilities/log_listener.py) on the
     listener host

2. **Set up the environment:**

   ```bash
   source $HOME/esp/v5.5/esp-idf/export.sh
   ```

   (Your ESP-IDF version may differ; please check your installation
   directory.)

3. **Navigate to the program's main directory:**

   ```bash
   cd $HOME/projects/syncspark/sparknode_sample_ota_app
   ```

4. **Add required dependencies:** Some projects require additional ESP-IDF
   standard components. For example, `sparknode_led_ring` needs the RGB LED
   strip component. To add it, run:

   ```bash
   idf.py add-dependency "espressif/led_strip^3.0.0"
   ```

   Please refer to each project's `README.md` for required system
   components.

5. **Configure the project:** Run:

   ```bash
   idf.py reconfigure
   ```

   to configure system parameters and set a custom partition table. For
   details, see [Appendix B: ESP32-CAM
   Configuration](#appendix-b-esp32-cam-configuration) and [Appendix C: Custom
   Partition Table](#appendix-c-custom-partition-table).

6. **Build the project:** Run:

   ```bash
   idf.py build
   ```

   in the top directory of the program (e.g.,
   `$HOME/projects/syncspark/sparknode_sample_ota_app`).

7. **Prepare the OTA binary:** Run:

   ```bash
   ../system/utilities/prep_ota_bin.sh <SparkNodeID>
   ```

   in the top directory of the program. Here, `<SparkNodeID>` is a number
   between 1 and 99 (we assign a `SparkNodeID` to each robot).

8. **Verify binary preparation:** Confirm that the `../system/ota` directory
   now contains the updated `*.bin`, `*.md5`, and the associated timestamp
   file.

9. **Reset the SparkNode:** Press the reset button on the SparkNode's
   ESP32-CAM.

To create a new ESP-IDF project from scratch, see [Appendix A: Additional Steps
to Create an ESP-IDF Project From
Scratch](#appendix-a-additional-steps-to-create-an-esp-idf-project-from-scratch).
