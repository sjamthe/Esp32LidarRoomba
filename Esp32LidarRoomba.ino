#include "src/Logger.h"

extern void setupNetwork(void);
extern void setupLidar();

void setup() {
    Serial.begin(115200);  // Debug serial debug output
    Serial.printf("\n\n\nStarting setup on %d core\n", xPortGetCoreID());
    // Setup WiFi, OTA, MicroROS and Telnet network services
    setupNetwork();
    setupLidar();

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
    */
}

void loop() {
    // Empty loop as tasks handle everything
    vTaskDelete(NULL);  // Delete setup and loop task
}
