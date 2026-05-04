# TLI4906 Hall Sensor Component

A comprehensive ESP32 component for TLI4906 hall effect sensor-based wheel turn counting and motion detection.

## Features

This component provides interrupt-driven hall sensor functionality for accurate wheel rotation measurement using two TLI4906 hall effect sensors:
- **Sensor 1 (Left wheel)**: Detects magnets attached to left wheel for tracking rotation
- **Sensor 2 (Right wheel)**: Detects magnets attached to right wheel for tracking rotation

## Key Functions

### Initialization and Setup
- `initialize_hall_sensors(const hall_sensor_config_t *config)` - Sets up GPIO interrupts for both sensors
- `deinitialize_hall_sensors(void)` - Clean up resources and remove interrupts

### Data Access
- `get_sensor_count(hall_sensor_id_t sensor_id)` - Returns total pulse count for specified sensor (SENSOR_1 or SENSOR_2)
- `get_time_since_last_pulse_us(hall_sensor_id_t sensor_id)` - Time since last pulse in microseconds, -1 if no pulse detected yet
- `get_sensor_states(bool *sensor_1_state, bool *sensor_2_state)` - Get current raw sensor states (for debugging)

### Motion Calculations
- `calculate_wheel_revolutions(hall_sensor_id_t sensor_id)` - Calculates complete wheel turns for specified wheel
- `calculate_wheel_angle_degrees(hall_sensor_id_t sensor_id)` - Current angle within revolution (0-360°) for specified wheel
- `calculate_wheel_rpm(hall_sensor_id_t sensor_id, uint32_t time_window_ms)` - Estimated RPM based on recent pulse timing
- `is_wheel_moving(hall_sensor_id_t sensor_id, uint32_t threshold_ms)` - Detects if specified wheel is currently rotating

### Utility Functions
- `reset_sensor_counts(void)` - Reset all counters to zero
- `wait_for_sensor_2_pulse(uint32_t timeout_ms)` - Blocking wait for next sensor 2 pulse

## Configuration

### Default Configuration
The component uses the following default settings:
```c
#define HALL_SENSOR_DEFAULT_MAGNETS_PER_REV    4
#define HALL_SENSOR_DEFAULT_DEBOUNCE_US        1000
#define HALL_SENSOR_DEFAULT_INVERT_LOGIC       false
```

### GPIO Setup
Configure your GPIO pins in `syncspark_config.h`:
```c
#define LEFT_WHEEL_SENSOR   GPIO_NUM_13  // Left wheel sensor (multiple magnets)
#define RIGHT_WHEEL_SENSOR  GPIO_NUM_14  // Right wheel sensor (multiple magnets)
```

### Sensor Logic
- **Default behavior**: Hall sensors are HIGH when no magnet is present, LOW when magnet is detected
- **Configurable**: Set `invert_logic = true` if your sensors work oppositely
- **Pull-up**: Component automatically enables GPIO pull-up resistors

## Usage Example

### Basic Initialization
```c
#include "tli4906.h"

// Create separate configurations for each sensor
hall_sensor_config_t sensor_1_config = HALL_SENSOR_DEFAULT_CONFIG(GPIO_NUM_13);
hall_sensor_config_t sensor_2_config = HALL_SENSOR_DEFAULT_CONFIG(GPIO_NUM_14);

// Initialize with both configurations
esp_err_t result = initialize_hall_sensors(&sensor_1_config, &sensor_2_config);

if (result == ESP_OK) {
    ESP_LOGI("MAIN", "Hall sensors initialized successfully");
} else {
    ESP_LOGE("MAIN", "Failed to initialize hall sensors: %s", esp_err_to_name(result));
}
```

### Custom Configuration
```c
// Configure left wheel sensor (sensor 1) with 16 magnets
hall_sensor_config_t sensor_1_config = {
    .sensor_gpio = GPIO_NUM_13,
    .magnets_per_revolution = 16,    // 16 magnets on left wheel
    .invert_logic = false,           // Magnet detected = LOW
    .debounce_time_us = 500          // 500μs debounce time
};

// Configure right wheel sensor (sensor 2) with 8 magnets
hall_sensor_config_t sensor_2_config = {
    .sensor_gpio = GPIO_NUM_14,
    .magnets_per_revolution = 8,     // 8 magnets on right wheel
    .invert_logic = false,
    .debounce_time_us = 500
};

initialize_hall_sensors(&sensor_1_config, &sensor_2_config);
```

### Reading Data
```c
// Get current counts for each wheel
uint32_t left_wheel_pulses = get_sensor_count(SENSOR_1);
uint32_t right_wheel_pulses = get_sensor_count(SENSOR_2);

// Calculate motion for left wheel
float left_revolutions = calculate_wheel_revolutions(SENSOR_1);
float left_angle = calculate_wheel_angle_degrees(SENSOR_1);
float left_rpm = calculate_wheel_rpm(SENSOR_1, 5000);  // RPM over last 5 seconds
bool left_moving = is_wheel_moving(SENSOR_1, 2000);    // No pulse for 2+ seconds = stopped

// Calculate motion for right wheel
float right_revolutions = calculate_wheel_revolutions(SENSOR_2);
float right_angle = calculate_wheel_angle_degrees(SENSOR_2);
float right_rpm = calculate_wheel_rpm(SENSOR_2, 5000);
bool right_moving = is_wheel_moving(SENSOR_2, 2000);

ESP_LOGI("HALL", "Left: %lu pulses, %.2f rev, %.1f°, %.1f RPM, Moving: %s",
         left_wheel_pulses, left_revolutions, left_angle, left_rpm,
         left_moving ? "Yes" : "No");
         
ESP_LOGI("HALL", "Right: %lu pulses, %.2f rev, %.1f°, %.1f RPM, Moving: %s",
         right_wheel_pulses, right_revolutions, right_angle, right_rpm,
         right_moving ? "Yes" : "No");
```

### Reset Counters
```c
// Reset all counters to zero
reset_sensor_counts();
```

## Integration

### CMakeLists.txt
The component is configured with:
```cmake
idf_component_register(SRCS "src/tli4906.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "driver" "esp_timer")
```

### Component Dependencies
The component requires:
- `driver` (for GPIO functions)
- `esp_timer` (for timing functions)
- `freertos` (for mutex and tasks)

## Technical Details

### Interrupt Handling
- Uses GPIO interrupts on falling edges (NEGEDGE)
- Interrupt handlers run in IRAM for fast response
- Built-in debouncing prevents false triggers from mechanical noise
- Thread-safe with mutex protection for shared data

### Timing Accuracy
- Uses `esp_timer_get_time()` for microsecond precision
- Calculates RPM based on time between consecutive pulses
- Tracks last pulse times for motion detection

### Memory Usage
- Minimal RAM footprint with single global data structure
- No dynamic memory allocation during operation
- Interrupt handlers optimized for speed

## Hardware Requirements

### TLI4906 Hall Sensors
- Digital hall effect sensors (TLI4906 or compatible)
- Output: HIGH when no magnet is present, LOW when magnet is detected
- Connect to 3.3V power and any available GPIO pins

### Magnets
- Small neodymium magnets mounted on each wheel
- Default configuration: 4 magnets per wheel (configurable)
- Magnets should be evenly spaced around the wheel circumference

### Wiring
```
TLI4906 VCC → ESP32 3.3V
TLI4906 GND → ESP32 GND  
TLI4906 OUT → ESP32 GPIO (with internal pull-up enabled)
```

Connect one TLI4906 sensor near the left wheel and another near the right wheel, with magnets attached to each wheel.

## Troubleshooting

### No Pulse Detection
- Check magnet placement and polarity
- Verify GPIO pin configuration in your code
- Ensure TLI4906 sensors are powered correctly (3.3V)
- Check wiring connections
- Verify magnets are close enough to sensors (typically < 5mm)

### False Triggers
- Increase debounce time (`debounce_time_us`)
- Check for electromagnetic interference
- Ensure stable power supply to sensors
- Verify magnet strength and positioning

### Inaccurate Counts
- Verify `magnets_per_revolution` setting matches actual magnet count
- Check for missed interrupts (reduce other interrupt load)
- Ensure magnets are evenly spaced on wheels
- Consider mechanical vibration effects
- Verify sensor mounting is secure

### Different Behavior per Wheel
- Each wheel can be monitored independently using SENSOR_1 and SENSOR_2
- Verify each sensor's GPIO configuration
- Check that magnets on both wheels are properly positioned