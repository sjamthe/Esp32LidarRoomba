#include "Logger.h"
#include <sys/time.h> // for timeval

#include "Constants.h"
#include "src/RPLidar/RPLidar.h"

RPLidar lidar(LIDAR_SERIAL, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_MOTOR_PIN); 

QueueHandle_t lidarQueue;
TaskHandle_t lidarTaskHandle = NULL;
bool lidar_status = false;

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
        if(lidarTask) {
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
}

bool checkLidarHealth() {
    // Check health
    RPLidar::DeviceHealth health;
    if (lidar.getHealth(health)) {
        logPrint(LOG_INFO, "RPLidar health status: %d, error code: %d", health.status, health.error_code);
        return health.status == 0;
    } else {
        logPrint(LOG_INFO, "Failed to retrieve RPLidar health status");
        return false;
    }
}

bool startLidarScan() {
    // Reset device before starting
    lidar.reset();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    logPrint(LOG_INFO,"Starting motor...");
    lidar.startMotor();
    delay(1000);  // Give motor time to reach speed

    // Start scan
    logPrint(LOG_INFO,"Starting scan...");
    if (!lidar.startExpressScan()) {
        logPrint(LOG_ERROR,"Failed to start express scan");
        lidar.stopMotor();
        return false;
    }
    logPrint(LOG_INFO,"Scan started successfully");
    return true;
}

void stopLidarScan() {
    lidar.stop();
    lidar.stopMotor();
    delay(1000);  // Give motor time to stop
}

void setupLidar() {
    // Initialize RPLidar
    logPrint(LOG_INFO,"Initializing RPLidar...");
    if (!lidar.begin()) {
        logPrint(LOG_INFO,"Failed to start RPLidar");
        return;
    }
    if(!checkLidarHealth()) {
        logPrint(LOG_INFO,"RPLidar health check failed");
        return;
    }
    logPrint(LOG_INFO,"RPLidar initialized");

    // Start scanning
    if(!startLidarScan()) {// TODO: move this based on roomba start commands.
        logPrint(LOG_INFO,"Failed to start RPLidar scan");
        return;
    }
    lidar_status = true;

    // Setup queue for IPC
    lidarQueue = xQueueCreate(1, LIDAR_BUFFER_SIZE);

    xTaskCreatePinnedToCore(
        lidarTask,
        "LidarTask",
        8192,
        NULL,
        2,
        &lidarTaskHandle,
        0  // Core 0
    );
}