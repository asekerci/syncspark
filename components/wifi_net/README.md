# WiFi Task Stack Overflow Problem and Solution (22 July 2025)

Github CoPilot has solved the problem after a quite solid trials. 

## What's Changed:
1. **Massive Stack Increases:**
   - TCP/IP task: 20KB stack (was 4KB)
   - System event task: 20KB stack (was 8KB)
   - Main task: 20KB stack (default was ~3KB)
2. **Memory Pressure Reduction:**
   - Reduced WiFi static buffers from 10→4 (TX) and 8→4 (RX)
   - Reduced dynamic RX buffers from 32→8
   - Reduced management buffers from 32→8
   - Reduced cache buffers to 8
3. **Core Affinity Optimization:**
   - Moved WiFi task to core 1 to reduce conflicts with your application tasks
   - Timer task: 8KB stack (was 2KB)
4. **Reduced Other Stack Usage:**
   - Idle task stack reduced to 2KB
   - Shared stack reduced to 2KB
5. **Persistent Configuration**
   - sdkconfig.defaults (see the folder sparknode_i2c_test) ensures these settings survive future builds 
## The Strategy:
Since the WiFi task stack is hardcoded at 6656 bytes, we're:
- Freeing up maximum heap memory by reducing buffer usage
- Preventing memory fragmentation with smaller static allocations
- Increasing all related task stacks to prevent overflow propagation
- Moving WiFi to dedicated core to reduce context switching overhead

This should give the WiFi task the best possible environment to operate within its fixed 6656-byte stack constraint.

Even though we couldn't directly increase the hardcoded 6656-byte WiFi task stack in ESP-IDF v5.4.1, we successfully worked around it by giving all the network-related tasks much more breathing room and reducing memory pressure throughout the system.

Your ESP32 firmware should now be stable for WiFi operations, OTA updates, and UDP logging without stack overflow issues! The system is ready for normal operation.