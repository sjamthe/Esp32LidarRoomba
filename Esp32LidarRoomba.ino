#include "src/Logger.h"

extern void setupNetwork(void);
extern void setupLidar();
extern void lidarTask(void *parameter);
TaskHandle_t lidarTaskHandle = NULL;


void setup() {
    Serial.begin(115200);  // Debug serial debug output
    Serial.printf("\n\n\nStarting setup on core %d\n", xPortGetCoreID());
    // Setup WiFi, OTA, MicroROS and Telnet network services
    //setupNetwork();
    //setupLidar();

    xTaskCreatePinnedToCore(
        lidarTask,
        "LidarTask",
        8192,
        NULL,
        1, // IMPORTANT: Don't drop priority for Lidar task. It needs to be responsive.
        &lidarTaskHandle,
        0  // Core 0
    );

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

//extern void getMeasurements();
/*unsigned long startMillis = 0;
unsigned long measurementCount = 0;
unsigned long rpsCount = 0;
unsigned long errorCount = 0;
unsigned long timeoutCount = 0;
bool firstMeasurement = true;*/

/*void loop() {
    //getMeasurements();
    vTaskDelay(pdMS_TO_TICKS(50));
}*/
