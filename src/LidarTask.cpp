#include "Logger.h"
#include <sys/time.h> // for timeval

#include "Constants.h"
#include "src/RPLidar/RPLidar.h"

RPLidar lidar(LIDAR_SERIAL, LIDAR_RX_PIN, LIDAR_TX_PIN, LIDAR_MOTOR_PIN); 

QueueHandle_t lidarQueue;
//TaskHandle_t lidarTaskHandle = NULL;
bool lidar_status = false;
unsigned long startMillis = 0;
unsigned long measurementCount = 0;
unsigned long rpsCount = 0;
unsigned long errorCount = 0;
unsigned long timeoutCount = 0;
bool firstMeasurement = true;
bool checkLidarHealth();
void setupLidar();

void getMeasurements() {

    size_t count = 0;
    MeasurementData measurements[RPLidar::EXPRESS_MEASUREMENTS_PER_SCAN];
    memset(measurements, 0, sizeof(measurements));

    //vTaskDelay(pdMS_TO_TICKS(50));
    sl_result ans = lidar.readMeasurement(measurements, count);
    if (ans == SL_RESULT_OK) {
        if (firstMeasurement) {
            startMillis = millis();
            logPrint(LOG_INFO,"First Lidar measurement received");
            firstMeasurement = false;
        }
        // sent to Front as only latest measurement matters.
        //xQueueSendToFront(lidarQueue, measurements, 0);
    } 

    //print stats every 10 seconds
    measurementCount+= count;

    for (int i=0; i<count; i++) {
        if(measurements[i].startFlag)   rpsCount++;
    }
    unsigned long now = millis();
    if ((now - startMillis) > 10000) {
        if (measurementCount == 0) {
            checkLidarHealth(); // Check
        }

        Serial.printf("Errors: %u. Timeouts: %u, Measurements: %u, Measurements per second: %04.0f, rps: %04.0f\n",
            errorCount, timeoutCount, measurementCount, measurementCount/((now-startMillis)/1000.0), rpsCount/((now-startMillis)/1000.0));
        Serial.printf("First measurement - Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
                        measurements[0].angle, measurements[0].distance, measurements[0].quality);
        Serial.printf("Last measurement - Angle: %.2f°, Distance: %.2fmm, Quality: %d\n", 
                        measurements[95].angle, measurements[95].distance, measurements[95].quality);
        startMillis = millis();
        measurementCount = 0;
        rpsCount = 0;
        errorCount = 0;
    }
}

void lidarTask(void *parameter) {

    setupLidar();

    for(;;) {
        //vTaskDelay(pdMS_TO_TICKS(50));
        getMeasurements();
        vTaskDelay(pdMS_TO_TICKS(5)); // This value is very critical. 10 works. 50 made scans stop totally. 0 crashes.
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
    //LIDAR_SERIAL.setRxBufferSize(1024); // incfease buffer size to 1KB - didn't help performance
    // Reset device before starting
    lidar.reset();
    delay(2000);  // Give it time to reset
    
    // Start motor with a clean delay sequence
    logPrint(LOG_INFO,"Starting motor...");
    lidar.startMotor();
    delay(1000);  // Give motor time to reach speed

    // Start scan
    logPrint(LOG_INFO,"Starting scan  on Core %d ...", xPortGetCoreID());
    //if (!lidar.startScan()) {
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
    lidarQueue = xQueueCreate(1, sizeof(MeasurementData*) * RPLidar::EXPRESS_MEASUREMENTS_PER_SCAN);
}