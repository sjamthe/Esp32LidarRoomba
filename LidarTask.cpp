#include "Logger.h"

const int LIDAR_BUFFER_SIZE = 256;
QueueHandle_t lidarQueue;

void processLidarData(char* buffer, int length) {
    // Process Lidar data
    sprintf(buffer, "Scan Time is %u", (unsigned int)millis());
}

void lidarTask(void *parameter) {
    //uint8_t roombaBuffer[ROOMBA_BUFFER_SIZE];
    char lidarBuffer[LIDAR_BUFFER_SIZE];
    
    while(1) {
        delay(120); // run every 100ms
        processLidarData(lidarBuffer, sizeof(lidarBuffer));
        
        if (xQueueSend(lidarQueue, lidarBuffer, 0) != pdPASS) {
            logPrint(LOG_ERROR, "Failed to send lidar data to queue");
        }
        else {
            logPrint(LOG_INFO, "%s on CPU Core %d",lidarBuffer, xPortGetCoreID());
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent watchdog triggers
    }
}

void setupLidar() {
    // Setup queue for IPC
    lidarQueue = xQueueCreate(10, LIDAR_BUFFER_SIZE);
}