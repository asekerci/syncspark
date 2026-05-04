# SparkNode Over-The-Air (OTA) Firmware Updater
## Overview

When the chip is programmed for the very first time, we flash `sparknode_ota_updater` in the `factory` partition via a USB connection. 

 Then, whenever the chip is rebooted, or power-cycled, it downloads a binary file and its corresponding MD5 checksum from the server specified in `network_config.h`. If the download is successful, the binary is written to the `ota_0` partition, the boot partition is set to `ota_0`, and the chip is restarted.

On subsequent boots, if the downloaded binary is properly configured (see `sparknode_sample_ota_app`[^1] for an example), the application changes the boot partition to `factory` to check the availability of updated firmware by running `sparknode_ota_updater` at every restart. 

Key Features are
- **Smart Checksum Caching**: Prevents unnecessary downloads
- **Robust Retry Logic**: 3-attempt mechanism for network failures  
- **Network Failure Fallback**: Boots existing firmware when remote access fails
- **Boot Partition Management**: Intelligent switching between factory and OTA partitions
- **Visual Status Indicators**: LED feedback for operation status
- **Remote Logging**: UDP-based debugging capabilities

[^1] The relevant code block is:
```
#ifdef OTA_UPDATE_ENABLED
    ESP_LOGI(TAG, "OTA update enabled");
    set_next_boot_to_factory();
#endif // OTA_UPDATE_ENABLED
```  
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
## Operational Details

### Main Algorithm

```
Algorithm 1: SparkNode OTA Updater Main Process
Input: WiFi credentials, HTTP server configuration
Output: Firmware updated and device rebooted to correct partition

1: procedure OTAUpdater()
2:    Initialize system (NVS, LEDs, WiFi, UDP logging)
3:    sparknode_id ← GetSparkNodeID()
4:    
5:    ▷ Phase 1: Fetch Remote Checksum
6:    for attempt = 1 to 3 do
7:        Fetch remote MD5 checksum from HTTP server
8:        if checksum fetched successfully AND length = 32 then
9:            break
10:       else
11:           Indicate error and continue
12:       end if
13:   end for
14:   
15:   ▷ Phase 1.5: Network Failure Fallback
16:   if checksum fetch failed after 3 attempts then
17:       stored_checksum ← RetrieveFromNVS("checksum")
18:       if stored_checksum exists then
19:           Log "Network issues detected, booting existing firmware"
20:           SetBootPartition(ota_0)
21:           IndicateSuccess()
22:           Restart()  ▷ Boot with existing firmware
23:       else
24:           Log "No stored checksum and unable to fetch remote"
25:           IndicateFailure()
26:           return
27:       end if
28:   end if
29:   
30:   ▷ Phase 2: Smart Caching Logic  
31:   stored_checksum ← RetrieveFromNVS("checksum") ▷ Only if not already retrieved
32:   if stored_checksum exists then
33:       if stored_checksum = remote_checksum then
34:           Log "No update needed, booting existing app"
35:           SetBootPartition(ota_0)
36:           IndicateSuccess()
37:           Restart()
38:       else
39:           Log "Update needed - checksums differ"
40:       end if
41:   else
42:       Log "No stored checksum - first time update"
43:   end if
33:   
34:   ▷ Phase 3: Download and Verify Firmware
35:   firmware_updated ← DownloadAndVerifyFirmware(remote_checksum)
36:   if firmware_updated = FALSE then
37:       IndicateFailure()
38:       return
39:   end if
40:   
41:   ▷ Phase 4: Complete OTA Process
42:   StoreInNVS("checksum", remote_checksum)
43:   SetBootPartition(ota_0)
44:   IndicateSuccess()
45:   Restart()
46: end procedure
```
### Sub-Algorithms

#### Fetch Remote Checksum with Retry

```
Algorithm 2: FetchRemoteChecksum
Input: url, maxAttempts = 3
Output: checksum string or NULL

1: function FetchRemoteChecksum(url, maxAttempts)
2:     for attempt = 1 to maxAttempts do
3:         Log attempt number
4:         result ← HTTP_GET(url)
5:         if result = SUCCESS then
6:             if checksum.length = 32 then
7:                 BlinkFlashLED(success_pattern)
8:                 return checksum
9:             else
10:                Log "Invalid checksum length"
11:                IndicateError()
12:            end if
13:        else
14:            Log "HTTP request failed"
15:            IndicateError()
16:        end if
17:    end for
18:    Log "Failed after all attempts"
19:    ▷ Main algorithm handles fallback to existing firmware
20:    return NULL
21: end function
```


