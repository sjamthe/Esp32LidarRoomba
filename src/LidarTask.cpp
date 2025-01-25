#include "Logger.h"
#include <sys/time.h> // for timeval

#include "Constants.h"
#include "src/RPLidar/RPLidar.h"

RPLidar lidar(LIDAR_SERIAL, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_MOTOR_PIN); 

QueueHandle_t lidarQueue;
TaskHandle_t lidarTaskHandle = NULL;
bool lidar_status = false;

void processLidarData(MeasurementData *measurements, size_t count, char* buffer, int length) {
    // Process Lidar data
    sprintf(buffer, "Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
			measurements[0].angle, measurements[0].distance, measurements[0].quality);
}


bool firstMeasurement = true;
void lidarTask(void *parameter) {
    char lidarBuffer[LIDAR_BUFFER_SIZE];
    MeasurementData measurements[lidar.EXPRESS_MEASUREMENTS_PER_SCAN];
    size_t count = 0;


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        sl_result ans = lidar.readMeasurement(measurements, count);
        if (ans == SL_RESULT_OK) {
            if (firstMeasurement) {
                logPrint(LOG_INFO,"First Lidar measurement received");
                firstMeasurement = false;
            }
            processLidarData(measurements, count, lidarBuffer, sizeof(lidarBuffer));
            // sent to Front as only latest measurement matters.
            xQueueSendToFront(lidarQueue, lidarBuffer, 0);
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
        1, // IMPORTANT: Don't drop priority for Lidar task. It needs to be responsive.
        &lidarTaskHandle,
        0  // Core 0
    );
}