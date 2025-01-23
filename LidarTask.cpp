#include "Logger.h"
#include <sys/time.h> // for timeval

const int LIDAR_BUFFER_SIZE = 256;
QueueHandle_t lidarQueue;

void processLidarData(char* buffer, int length) {
    // Process Lidar data
    //temp - fill with time for now
    timeval _timeval;
    gettimeofday(&_timeval, NULL); 
    struct tm timeinfo;
    localtime_r(&_timeval.tv_sec, &timeinfo);

    snprintf(buffer, 17, "%02d:%02d:%02d.%06d",
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,_timeval.tv_usec );
}

void lidarTask(void *parameter) {
    //uint8_t roombaBuffer[ROOMBA_BUFFER_SIZE];
    char lidarBuffer[LIDAR_BUFFER_SIZE];
    
    while(1) {
        processLidarData(lidarBuffer, sizeof(lidarBuffer));
        
        //if (xQueueSend(lidarQueue, lidarBuffer, 0) != pdPASS) {
        if (xQueueSendToFront(lidarQueue, lidarBuffer, 0) != pdPASS) {
          // Fails a lot but looks like problem on ros2-agent side
           // logPrint(LOG_ERROR, "Failed to send lidar data to queue");
        }
        else {
            //logPrint(LOG_INFO, "%s on CPU Core %d",lidarBuffer, xPortGetCoreID());
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent watchdog triggers
    }
}

void setupLidar() {
    // Setup queue for IPC
    lidarQueue = xQueueCreate(1, LIDAR_BUFFER_SIZE);
}