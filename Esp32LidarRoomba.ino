#include "Logger.h"

extern void setupNetwork(void);

// Task handles
TaskHandle_t roombaTaskHandle = NULL;
TaskHandle_t lidarTaskHandle = NULL;
TaskHandle_t networkTaskHandle = NULL;

extern void networkTask(void *parameter);

void setup() {
    // Initialize serial ports
    Serial.flush(); // to clear serial?
    delay(1000);
    Serial.begin(115200);  // Debug serial debug output
    Serial.printf("Starting setup on %d core\n", xPortGetCoreID());
    // Setup WiFi, OTA, and other network services
    setupNetwork();
    /*
    // Create tasks
    xTaskCreatePinnedToCore(
        roombaTask,
        "RoombaTask",
        8192,
        NULL,
        2,
        &roombaTaskHandle,
        0  // Core 0
    );
    
    xTaskCreatePinnedToCore(
        lidarTask,
        "LidarTask",
        8192,
        NULL,
        2,
        &lidarTaskHandle,
        0  // Core 0
    );
    */
    xTaskCreatePinnedToCore(
        networkTask,
        "NetworkTask",
        8192,
        NULL,
        1,
        &networkTaskHandle,
        1  // Core 1
    );
}

void loop() {
    // Empty loop as tasks handle everything
    vTaskDelete(NULL);  // Delete setup and loop task
}
