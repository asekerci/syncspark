# SparkNode MQTT Control Commands

## Table of Contents

- [Communication](#communication)
- [Configuration Commands](#configuration-commands)
- [Movement Commands](#movement-commands)
- [IMU Sensor System](#imu-sensor-system)
  - [IMU Data Topics](#imu-data-topics)
    - [Orientation Data](#orientation-data)
    - [Orientation Data by Sensor Mode](#orientation-data-by-sensor-mode)
  - [IMU Calibration Commands](#imu-calibration-commands)
  - [IMU Control Commands](#imu-control-commands)
- [System Commands](#system-commands)
- [Command Behavior](#command-behavior)
- [Example Usage](#example-usage)
  - [Movement Examples](#movement-examples)
  - [Configuration Examples](#configuration-examples)
  - [Sensor Control Examples](#sensor-control-examples)
  - [IMU Calibration Examples](#imu-calibration-examples)
  - [Navigation Examples](#navigation-examples)
  - [Scripted Command Sequences](#scripted-command-sequences)
- [Implementation Notes](#implementation-notes)
- [Per-SparkNode Configuration](#per-sparknode-configuration)
- [Appendix A: MQTT System](#appendix-a-mqtt-system)

## Communication

-   **Topic to send commands to an individual SparkNode**: `arena/sparknodeXX/cmd` (where `sparknodeXX` is the ID 
   of the SparkNode you want to control). Example:
```bash
      mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "drive forward 32 5000"
```
-   **Topic to send commands to all SparkNodes (broadcast)**: `arena/all/cmd`
-   **Status responses appear on**: `arena/sparknodeXX/status`. The command to listen to these messages is:
```bash
      mosquitto_sub -h $broker_address -t 'arena/+/status'
```

## Configuration Commands

-   `config show` - Display the current SparkNode configuration
-   `config set drive_kick <speed> <duration>` - Set drive kick parameters
    (speed: 16-63, duration: 50-500ms)
-   `config set turn_kick <speed> <duration>` - Set turn kick parameters (speed:
    10-63, duration: 25-500ms)
-   `config set wheel_calibration <left> <right>` - Set wheel speed calibration
    factors (0.5-2.0)
-   `config set default_speed <speed>` - Set the default movement speed (0-63)

## Movement Commands

-   `drive forward <speed> <duration>` - Drive forward at the specified speed (0-63)
    for the given duration (ms)
-   `drive reverse <speed> <duration>` - Drive backward at the specified speed
    (0-63) for the given duration (ms)
-   `turn left <speed> <duration>` - Turn left at the specified speed (0-63) for
    the given duration (ms)
-   `turn right <speed> <duration>` - Turn right at the specified speed (0-63) for
    the given duration (ms)
-   `rotate to <angle> [speed] [step_ms] [delay_ms] [tolerance]` - Rotate to a specific magnetometer heading (0-360 degrees)
    - `angle` - Target heading in degrees (0-360, required)
    - `speed` - Fixed rotation speed (0-63, default: 15)
    - `step_ms` - Duration of each rotation step in milliseconds (minimum: 5, default: 10)
    - `delay_ms` - Delay between heading checks in milliseconds (minimum: 100, default: 300)
    - `tolerance` - Acceptable heading error in degrees (minimum: 5.0, default: 5.0)
    - Uses fixed speed for all rotation steps
-   `turn to <angle> [speed]` - Continuously turn to target heading with proportional speed control
    - `angle` - Target heading in degrees (0-360, required)
    - `speed` - Maximum rotation speed (0-63, default: 20)
    - Uses vector averaging of 3 magnetometer samples per iteration for stable heading
    - Proportional speed ramping based on heading error (minimum speed: 12)
    - Stops when within 5° tolerance
-   `stop` - Immediately halt both motors (brake then coast)

## IMU Sensor System 

### IMU Data Topics

SparkNodes publish sensor data to specific topics when enabled:

#### Orientation Data

-   **Topic**: `arena/sparknodeXX/orientation`
-   **Publishing**: Automatically published when sensor streaming is enabled via `sensor_stream start`
-   **Format**: JSON payload with Euler angles (in degrees) (example payload: `{"yaw":85.32,"pitch":2.15,"roll":-1.03}`)
-   **Update rate**: Configurable via `sensor_stream start [period_ms]` (minimum: 20ms)
-   **Data source**: Computed using Madgwick AHRS algorithm for sensor fusion
-   **Subscribe to all nodes**: 
```bash
      mosquitto_sub -h <broker_ip> -t 'arena/+/orientation'
```

#### Orientation Data by Sensor Mode

The orientation data varies based on the active sensor mode:

-   **`mag_accel_gyro`** (default) - Full 9-DOF sensor fusion
    - Provides: yaw (0-360°), pitch, and roll from magnetometer + accelerometer + gyroscope
    - Uses Madgwick filter for accurate orientation in 3D space
    - Best for complete orientation tracking

-   **`accel_gyro`** - 6-DOF sensor fusion (no magnetometer)
    - Provides: yaw (with +90° alignment offset), pitch, and roll from accelerometer + gyroscope
    - Suitable when magnetic interference is present
    - Note: Yaw may drift over time without magnetometer correction

-   **`mag`** - Magnetometer-only mode
    - Provides: yaw from magnetometer heading only
    - pitch and roll are set to 0.0
    - Useful for heading-only applications

-   **`mag_accel`** - Magnetometer + accelerometer fusion
    - Provides: yaw from magnetometer, pitch and roll from accelerometer
    - No gyroscope drift correction

### IMU Calibration Commands

-   `calibrate gyro` - Recalibrate gyroscope bias and save to NVS (non-volatile storage)
-   `calibrate mag <mode>` - Recalibrate magnetometer and save calibration data to NVS
    - `hard` - 2D calibration with flat rotation (robot automatically rotates)
    - `soft` - 3D calibration with figure-8 movement (manual movement required)
    - `capture [samples] [delay_ms]` - Capture raw magnetometer data for external calibration
        - samples: 100-1200 (default: 500)
        - delay_ms: 10-300ms between samples (default: 50ms)
        - Publishes each sample to MQTT for logging and external processing
        - Use for high-precision 3D calibration with external fitting tools
    - `apply <bias_x> <bias_y> <bias_z> <m00> <m01> <m02> <m10> <m11> <m12> <m20> <m21> <m22> [version]` - Apply externally computed calibration
        - Takes 3 hard-iron bias values and 9 soft-iron matrix values
        - Optional version parameter (1 or 3, default: 3)
        - Validates all values are finite before saving to NVS
-   `show calibrations` - Display current magnetometer and gyroscope calibration data
    (includes hard-iron bias, soft-iron matrix diagonal, and version info)

### IMU Control Commands

-   `sensor_loop <mode> [iterations] [delay_ms]` - Control the sensor data collection loop
    - `start [delay_ms]` - Run the sensor loop continuously
    - `stop` - Stop sensor data collection
    - `counted <iterations> [delay_ms]` - Run the sensor loop for a specified number of iterations
    - **Note**: The time interval between sensor readings is controlled by the `g_sensor_loop_delay_ms` global variable (default: 50ms)
-   `sensor_stream <start|stop|mode> [options]` - Control periodic sensor data streaming
    - `start [period_ms]` - Start streaming sensor data to `arena/sparknodeXX/orientation` topic (minimum period: 20ms)
    - `stop` - Stop sensor data streaming
    - `mode <mag_accel_gyro|accel_gyro|mag|mag_accel>` - Set which sensors to use for orientation computation
        - `mag_accel_gyro` - Full 9-DOF fusion (magnetometer + accelerometer + gyroscope) - default, best accuracy
        - `accel_gyro` - 6-DOF fusion (accelerometer + gyroscope only) - no mag, may drift
        - `mag` - Magnetometer-only heading (yaw only, pitch/roll = 0)
        - `mag_accel` - Magnetometer + accelerometer (no gyroscope)
-   `show heading [count] [delay_ms]` - Get the current magnetometer heading without moving the robot
    - `count` - Number of samples to average (default: 1)
    - `delay_ms` - Delay between samples in milliseconds (minimum: 10, default: 50)
    - Returns averaged heading with circular standard deviation and magnetic field magnitude statistics

## System Commands

-   `reboot` - Restart the SparkNode

## Command Behavior

**Drive/Turn Motion**:

-   Uses ramped acceleration/deceleration for smooth movement
-   Speed parameter: 0-63 (maps to PWM duty cycle: 0-100%)
-   Default speed: 32 (≈51% PWM duty cycle)
-   Runs in separate FreeRTOS tasks to keep MQTT responsive
-   Automatically stops and brakes at the end of the duration
-   **Kick-start mechanism**: Automatically applies higher initial torque to
    overcome static friction
-   **Sequencing**: Wait for command completion before sending next movement command

**Magnetometer-Based Navigation**:

-   **Show Heading**: Returns current magnetometer heading with optional multi-sample averaging and statistical analysis (circular standard deviation and magnetic field magnitude)
-   **Turn To (Proportional Control)**: Uses vector averaging of 3 magnetometer samples per iteration for stable heading. Speed is proportional to heading error with minimum speed of 12. Stops when within 5° tolerance
-   **Rotate To (Fixed Speed Control)**: Uses fixed rotation steps at constant speed. Default tolerance is 5° with a 100-iteration timeout. Default speed is 15, step duration is 10ms, and delay between checks is 300ms
-   **Closed-loop heading control**: Both turn commands use a calibrated magnetometer for accurate directional control
-   **Real-time feedback**: Publishes the current heading and progress during rotation
-   **Automatic sensor activation**: Temporarily enables the sensor loop during rotation for heading updates

**Stop Command**:

-   Immediate effect - applies brake mode briefly, then coast
-   Useful for emergency stops or interrupting ongoing motion

**Configuration System**:

-   **Per-SparkNode customization**: Each robot can have individual motor
    parameters
-   **Runtime configuration**: Settings are temporary and reset to defaults on
    reboot (no NVS storage)
-   **Motor calibration**: Compensates for manufacturing differences between
    left and right motors
-   **Kick parameters**: Customizes startup torque for different motor break-in
    states
-   **Real-time updates**: Configuration changes take effect immediately

**Sensor Control**:

-   **Sensor loop modes**: Controls when and how often IMU sensor data is collected and processed
-   **Real-time control**: Starts/stops sensor fusion processing on demand
-   **Counted mode**: Runs the sensor loop for a specific number of iterations for testing
-   **Sensor streaming**: Automatically publishes orientation data to `arena/sparknodeXX/orientation` topic
-   **Stream modes**: Flexible selection of which sensors to use for orientation computation:
    - **mag_accel_gyro**: Full 9-DOF Madgwick sensor fusion (most accurate)
    - **accel_gyro**: 6-DOF fusion without magnetometer (useful in magnetically noisy environments)
    - **mag**: Magnetometer-only heading (simple compass functionality)
    - **mag_accel**: Mag + accel fusion (no gyroscope)
-   **Data format**: JSON Euler angles in degrees: `{"yaw":xx.xx,"pitch":xx.xx,"roll":xx.xx}`
-   **Update rate**: Configurable from 20ms (50Hz) upward via period_ms parameter
-   **Sensor fusion algorithm**: Uses Madgwick AHRS filter for quaternion-based orientation estimation

**IMU Calibration**:

-   **Gyroscope calibration**: The robot must be stationary during the calibration process
-   **Magnetometer calibration**: Multiple modes available:
    - **Hard-iron (2D)**: Robot automatically rotates flat for calibration of hard-iron distortions
    - **Soft-iron (3D)**: Requires manual figure-8 movement for full 3D soft-iron distortion calibration
    - **Capture mode**: Collects raw magnetometer samples and publishes them via MQTT for external processing
    - **Apply mode**: Accepts externally computed calibration parameters (hard-iron bias and soft-iron matrix)
-   **External calibration workflow**: Capture raw data → process on host with advanced algorithms → apply results
-   **Persistent storage**: Calibration data is automatically saved to NVS and restored on reboot
-   **Status feedback**: Publishes success/failure status with calibration values
-   **Calibration inspection**: Use `show calibrations` to view current active calibration parameters

**Status Updates**:

-   Each accepted command publishes a confirmation with speed and duration details
-   Configuration changes publish acknowledgment messages
-   IMU calibration commands provide detailed feedback on calibration results
-   Unknown commands receive comprehensive help text

## Example Usage

Using `mosquitto_pub` or any MQTT client:

### **Movement Examples**

```bash
# Drive forward at speed 32 for 5000 milliseconds (5 seconds)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "drive forward 32 5000"

# Turn right at speed 25 for 1500 milliseconds 
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn right 25 1500"

# Slow reverse movement at speed 15 for 2000ms
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "drive reverse 15 2000"

# Fast turn at speed 45 for 800ms
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn left 45 800"

# Rotate to 90 degrees (east) using default parameters
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 90"

# Rotate to 180 degrees (south) with custom speed and precision
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 180 25 30 200 3.0"

# Emergency stop
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "stop"

# Show current heading without moving
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "show heading"

# Turn to 90 degrees using proportional control
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn to 90"

# Turn to 180 degrees with custom max speed
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn to 180 30"

# Broadcast command to all sparknodes
mosquitto_pub -h <broker_ip> -t "arena/all/cmd" -m "stop"
```

### **Configuration Examples**

```bash
# Show current configuration
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "config show"

# Show current IMU calibration data
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "show calibrations"

# Set gentler drive kick for well-broken-in motors
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "config set drive_kick 25 100"

# Set stronger turn kick for stubborn motors  
mosquitto_pub -h <broker_ip> -t "arena/sparknode07/cmd" -m "config set turn_kick 40 250"

# Compensate for motor differences (left motor 5% weaker)
mosquitto_pub -h <broker_ip> -t "arena/sparknode03/cmd" -m "config set wheel_calibration 1.05 1.0"

# Set default movement speed
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "config set default_speed 28"
```

### **Sensor Control Examples**

```bash
# Start continuous sensor data collection
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_loop start"

# Start continuous sensor loop with custom delay (100ms)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_loop start 100"

# Stop sensor data collection  
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_loop stop"

# Run sensor loop for exactly 100 iterations
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_loop counted 100"

# Start sensor streaming with default period
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_stream start"

# Start sensor streaming with 100ms period
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_stream start 100"

# Set sensor stream mode to magnetometer only
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_stream mode mag"

# Set sensor stream mode to accelerometer and gyroscope
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_stream mode accel_gyro"

# Stop sensor streaming
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "sensor_stream stop"

# Broadcast sensor start to all sparknodes
mosquitto_pub -h <broker_ip> -t "arena/all/cmd" -m "sensor_loop start"

# Subscribe to orientation data from a specific sparknode
mosquitto_sub -h <broker_ip> -t "arena/sparknode01/orientation"

# Subscribe to orientation data from all sparknodes
mosquitto_sub -h <broker_ip> -t "arena/+/orientation"

# Monitor both status and orientation data
mosquitto_sub -h <broker_ip> -t "arena/sparknode01/#"
```

### **IMU Calibration Examples**

```bash
# Recalibrate gyroscope (robot should be stationary during calibration)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate gyro"

# Recalibrate magnetometer using 2D (flat rotation) calibration
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate mag hard"

# Recalibrate magnetometer using 3D (figure-8) calibration
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate mag soft"

# Capture raw magnetometer data for external calibration (default 500 samples, 50ms delay)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate mag capture"

# Capture 800 magnetometer samples with 100ms delay
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate mag capture 800 100"

# Apply externally computed calibration (example values)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate mag apply -12.5 8.3 -15.2 1.02 0.01 -0.02 0.01 0.98 0.03 -0.02 0.03 1.01 3"

# Broadcast gyro calibration to all sparknodes
mosquitto_pub -h <broker_ip> -t "arena/all/cmd" -m "calibrate gyro"

# Example of error handling for unsupported sensor type
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "calibrate accel"
# Response: "Unknown sensor type: accel (supported: gyro, mag)"
```

### **Navigation Examples**

```bash
# Show current heading without moving the robot (single sample)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "show heading"

# Show average heading over 10 samples with 100ms delay
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "show heading 10 100"

# Show average heading over 5 samples with 200ms delay  
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "show heading 5 200"

# Turn to 90 degrees (east) using proportional control (default speed)
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn to 90"

# Turn to 180 degrees (south) with custom maximum speed
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "turn to 180 30"

# Rotate to 90 degrees (east) using default parameters
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 90"

# Rotate to 180 degrees (south) with custom speed
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 180 25"

# Rotate to 270 degrees (west) with full parameter control
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 270 30 50 300 5.0"
# Parameters: target=270°, speed=30, step=50ms, delay=300ms, tolerance=5.0°

# Rotate to north (0 degrees) with custom speed
mosquitto_pub -h <broker_ip> -t "arena/sparknode01/cmd" -m "rotate to 0 20"

# Broadcast rotation command to all sparknodes
mosquitto_pub -h <broker_ip> -t "arena/all/cmd" -m "rotate to 90"
```

### **Scripted Command Sequences**

When sending multiple movement commands in a script, you **must wait** for each command to complete before sending the next one. Here is a complete example script:

```bash
#!/bin/bash
# SparkNode Sequential Movement Script
mqtt_broker="192.168.11.100"
sparknode="sparknode01"

echo "Starting movement sequence for $sparknode..."

# Show current configuration first
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "config show"
sleep 2

echo "=== Movement Sequence ==="

# Drive forward for 5 seconds
echo "Driving forward..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive forward 32 5000"
sleep 6  # Wait: 5 seconds (command duration) + 1 second (safety buffer)

# Turn right for 1 second
echo "Turning right..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn right 30 1000"
sleep 2  # Wait: 1 second (command duration) + 1 second (safety buffer)

# Drive reverse for 3 seconds
echo "Driving reverse..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "drive reverse 25 3000"
sleep 4  # Wait: 3 seconds (command duration) + 1 second (safety buffer)

# Turn left for 1.5 seconds
echo "Turning left..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn left 30 1500"
sleep 3  # Wait: 1.5 seconds (command duration) + 1.5 seconds (safety buffer)

echo "Movement sequence complete!"

# Emergency stop example
# mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "stop"
```

**Sensor Data Collection Script Example**:

```bash
#!/bin/bash
# SparkNode Sensor Collection Script
mqtt_broker="192.168.11.100"
sparknode="sparknode01"

echo "Starting sensor data collection for $sparknode..."

# Start continuous sensor data collection
echo "Starting continuous sensor loop..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop start"

# Let it run for 30 seconds
sleep 30

echo "Stopping sensor loop..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop stop"

echo "Collecting 50 sensor samples..."
# Collect exactly 50 samples  
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop counted 50"

# Wait for completion (allow sufficient time for samples to complete)
sleep 12

echo "Sensor collection complete!"

# Example: Subscribe and monitor orientation data
mosquitto_sub -h $mqtt_broker -t "arena/+/orientation" -v &
# Output will show messages like:
# arena/sparknode01/orientation {"yaw":85.32,"pitch":2.15,"roll":-1.03}
```

**Navigation Script Example**:

```bash
#!/bin/bash
# SparkNode Navigation Script - Rotate through cardinal directions
mqtt_broker="192.168.11.100"
sparknode="sparknode01"

echo "Starting navigation sequence for $sparknode..."

# Start continuous sensor data collection for heading feedback
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop start"
sleep 2

# First, check current heading
echo "Checking current heading..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "show heading"
sleep 2

echo "=== Navigation Sequence ==="

# Use "turn to" for smoother proportional control
# Turn to North (0 degrees)
echo "Turning to North (0°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to 0 25"
sleep 15  # Allow time for turn to complete

# Turn to East (90 degrees)
echo "Turning to East (90°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to 90 25"
sleep 15  # Allow time for turn to complete

# Turn to South (180 degrees) 
echo "Turning to South (180°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to 180 25"
sleep 15  # Allow time for turn to complete

# Turn to West (270 degrees)
echo "Turning to West (270°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to 270 25"
sleep 15  # Allow time for turn to complete

# Return to North
echo "Returning to North (0°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "turn to 0 25"
sleep 15  # Allow time for turn to complete

# Final heading check
echo "Final heading check..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "show heading"
sleep 2

echo "Navigation sequence complete!"

# Stop sensor loop
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop stop"
```

**Alternative using "rotate to" for precise step control**:

```bash
#!/bin/bash
# SparkNode Precision Navigation Script - Using rotate to with precise parameters
mqtt_broker="192.168.11.100"
sparknode="sparknode01"

echo "Starting precision navigation sequence for $sparknode..."

# Start continuous sensor data collection for heading feedback
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop start"
sleep 2

echo "=== Precision Navigation Sequence ==="

# Rotate to North (0 degrees) with custom parameters
echo "Rotating to North (0°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "rotate to 0 15 10 300 5.0"
sleep 15  # Allow time for rotation to complete

# Rotate to East (90 degrees)
echo "Rotating to East (90°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "rotate to 90 15 10 300 5.0"
sleep 15  # Allow time for rotation to complete

# Rotate to South (180 degrees) 
echo "Rotating to South (180°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "rotate to 180 15 10 300 5.0"
sleep 15  # Allow time for rotation to complete

# Rotate to West (270 degrees)
echo "Rotating to West (270°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "rotate to 270 15 10 300 5.0"
sleep 15  # Allow time for rotation to complete

# Return to North
echo "Returning to North (0°)..."
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "rotate to 0 15 10 300 5.0"
sleep 15  # Allow time for rotation to complete

echo "Precision navigation sequence complete!"

# Stop sensor loop
mosquitto_pub -h $mqtt_broker -t "arena/$sparknode/cmd" -m "sensor_loop stop"
```

**⚠️ Critical Timing Rule**: 
```bash
# For each movement command: sleep (command_duration_ms/1000 + safety_buffer)
mosquitto_pub -h $broker -t "arena/sparknode01/cmd" -m "drive forward 32 5000"
sleep 6  # 5 seconds (movement) + 1 second (safety buffer)

# For rotate to commands: allow ~10-15 seconds depending on angle difference and parameters
mosquitto_pub -h $broker -t "arena/sparknode01/cmd" -m "rotate to 90 15 10 300 5.0"
sleep 15  # Allow time for magnetometer-based rotation to complete

# For turn to commands: allow ~15-30 seconds depending on angle difference
mosquitto_pub -h $broker -t "arena/sparknode01/cmd" -m "turn to 90 25"
sleep 15  # Allow time for proportional turn control to complete
```

## Implementation Notes

-   **Non-blocking**: Drive/turn commands spawn tasks, so multiple commands can
    be queued
-   **Motor control**: Uses the DRV8830 dual motor driver with a configurable
    kick-start mechanism
-   **IMU integration**: ICM-20948 9-DOF sensor with gyroscope and magnetometer calibration
-   **Sensor control**: Provides real-time control of IMU sensor data collection and processing loops
-   **Persistent calibration**: IMU calibration data is stored in NVS for consistent performance across reboots
-   **Magnetometer navigation**: Implements closed-loop heading control using a calibrated magnetometer with automatic correction
-   **Error handling**: Publishes status on successful parsing; unknown commands
    receive help text
-   **Safety**: The stop command provides immediate motor shutdown capability
-   **Static friction compensation**: Automatic kick-start ensures reliable
    motor startup
-   **Individual motor calibration**: Provides left/right wheel speed compensation for
    straight-line accuracy
-   **Fleet management**: Each SparkNode can be optimized for its specific motor
    and sensor characteristics

## Per-SparkNode Configuration

The system supports individual configuration for each SparkNode to handle motor
manufacturing variations:

**Configuration Parameters**:

-   **Drive kick**: Initial torque pulse for straight movement (speed and
    duration)
-   **Turn kick**: Initial torque pulse for turning (usually stronger than the drive
    kick)
-   **Wheel calibration**: Speed correction factors to compensate for motor
    differences
-   **Default speed**: Fallback speed when not specified in commands

**Usage Strategy**:

-   **Well-broken-in SparkNodes**: Use gentler kick parameters (lower
    speed and shorter duration)
-   **New/stubborn SparkNodes**: Use stronger kicks for reliable startup
-   **Motor matching**: Use wheel calibration factors to ensure straight movement
    despite motor variations
-   **Fleet optimization**: Test and configure each robot individually for optimal
    performance

## Appendix A: MQTT System

We use a Raspberry Pi 5 as the broker (TODO: network diagram). Here are the
installation notes:

``` bash
sudo apt update
sudo apt install mosquitto mosquitto-clients -y
```

where `mosquitto` is the broker (server) and `mosquitto-clients` are
command-line tools for publishing and subscribing (useful for testing).

Check that the configuration file `/etc/mosquitto/mosquitto.conf` has the proper
entries:

```
    # Place your local configuration in /etc/mosquitto/conf.d/
    #
    # A full description of the configuration file is at
    # /usr/share/doc/mosquitto/examples/mosquitto.conf.example

    pid_file /run/mosquitto/mosquitto.pid

    persistence true
    persistence_location /var/lib/mosquitto/

    log_dest file /var/log/mosquitto/mosquitto.log

    include_dir /etc/mosquitto/conf.d

    listener 1883 0.0.0.0
    allow_anonymous true
```
After the installation, start running the server:

``` bash
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

Check that it is running:

``` bash
sudo systemctl status mosquitto
```
