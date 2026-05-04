Work in progress: Code is not written yet. 
## Approach: LED Ring Detection & ID Recognition

Since each SparkNode displays a unique De Bruijn sequence on its LED ring, we can:
1. **Capture images** with the ESP32-CAM
2. **Detect circular LED patterns** (16 LEDs in a ring)
3. **Read the color sequence** (Red=0, Green=1, Blue=2)
4. **Match against known De Bruijn sequences** to identify the SparkNode ID
5. **Estimate position/distance** based on ring size and location in frame

## **Implementation Plan**

**1. Camera Capture Component:**
````c
// New component: vision
void capture_frame_for_tracking();
esp_err_t detect_sparknode_rings(uint8_t* image_buffer, sparknode_detection_t* detections);
````

**2. Computer Vision Pipeline:**
````c
typedef struct {
    uint8_t sparknode_id;
    float x_position;      // Normalized coordinates (0.0-1.0)
    float y_position;
    float distance_estimate; // Based on ring size
    uint8_t confidence;    // Detection confidence (0-100)
} sparknode_detection_t;
````

**3. Detection Algorithm:**
- Use **circular Hough transform** or **contour detection** to find ring-shaped objects
- **Sample LED colors** at 16 positions around detected circles
- **Convert RGB → color index** (0,1,2)
- **Match sequence** against your `debruijn_sequences` array
- **Calculate relative position** from camera center

**4. Integration with your existing code:**
````c
void task_sparknode_tracker(void *pvParameters) {
    sparknode_detection_t detections[MAX_DETECTIONS];
    
    while (1) {
        int count = detect_sparknode_rings(camera_buffer, detections);
        
        for (int i = 0; i < count; i++) {
            ESP_LOGI("Tracker", "Detected SparkNode ID=%d at (%.2f,%.2f) distance=%.2f", 
                detections[i].sparknode_id, 
                detections[i].x_position, 
                detections[i].y_position,
                detections[i].distance_estimate);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Track at 2Hz
    }
}
````

Would you like me to help you implement:
1. **Camera capture setup** for the ESP32-CAM?
2. **Basic circle detection** algorithm?
3. **Color sequence reading** from detected rings?
4. **Integration** with your existing SparkNode system?

Which part would you like to start with?
