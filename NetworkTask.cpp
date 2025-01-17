#include <ArduinoOTA.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "WifiSetup.h" // Includes wifi information
# include "Logger.h"

void syncTime() {
  struct tm timeinfo;
  char timeStringBuff[50];  // Make sure this is large enough for your format

  // Configure timezone and NTP
  configTzTime("PST8PDT,M3.2.0,M11.1.0", "pool.ntp.org", "time.nist.gov");
  
  // Wait for sync
  time_t now = time(nullptr);
  int retry = 0;
  while (now < 8 * 3600 * 2) {
    logPrint(LOG_INFO, "Waiting for NTP time sync...");
    delay(500);
    now = time(nullptr);
    if (++retry > 20) break;  // Timeout after 10 seconds
  }
  
  // Get and print local time
  if (getLocalTime(&timeinfo)) {
    logPrint(LOG_INFO, "Local time: ");
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S %Z", &timeinfo);
    logPrint(LOG_INFO, "Current time: %s", timeStringBuff);  
  } else {
    logPrint(LOG_ERROR, "Failed to obtain time");
  }
}

void setupWifi() {
    
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname); 
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    logPrint(LOG_WARN, "WiFi Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  syncTime(); // Initialize correct time for logging.

  // Enable mDNS responder
  if(!MDNS.begin(hostname)) {
    logPrint(LOG_ERROR, "Error setting up MDNS responder!");
  }

  logPrint(LOG_INFO, "WiFi connected to: %s", String(ssid));
  logPrint(LOG_INFO, "IP address: %s", WiFi.localIP().toString());
  logPrint(LOG_INFO, "Hostname: %s", WiFi.getHostname());
}

void shutdownGracefully() {
  // close all open connections etc for OTA update.
  logPrint(LOG_INFO,"Shutting down connections ...");
}

void setupOTA() {
  // OTA Setup
    ArduinoOTA.onEnd([]() {
    logPrint(LOG_INFO, "OTA update complete. About to reboot.");
    shutdownGracefully();
    delay(1000);  // Give some time for the OTA response to be sent
    ESP.restart(); // Programmatically trigger a reset
    });

  // Other OTA event handlers
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    logPrint(LOG_INFO, "\nStart OTA updating %s", type);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    logPrint(LOG_INFO, "OTA update progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    logPrint(LOG_INFO, "Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) logPrint(LOG_ERROR, "ArduinoOTA Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logPrint(LOG_ERROR, "ArduinoOTA Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logPrint(LOG_ERROR, "ArduinoOTA Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logPrint(LOG_ERROR, "ArduinoOTA Receive Failed");
    else if (error == OTA_END_ERROR) logPrint(LOG_ERROR, "ArduinoOTA End Failed");
  });

//set password for OTA
  ArduinoOTA.setPassword(otaPassword);

  ArduinoOTA.begin(); // default port 3232
  logPrint(LOG_INFO, "OTA service started");
}

void setupNetwork() {
    // Connect to WiFi
    setupWifi();
    
    // Start OTA server
    setupOTA(); 
}

// Network task running on Core 0
void networkTask(void *parameter) {
    //uint8_t roombaBuffer[ROOMBA_BUFFER_SIZE];
    //uint8_t lidarBuffer[LIDAR_BUFFER_SIZE];
    
    while(1) {
        // Handle OTA updates
        ArduinoOTA.handle();
        
        // Handle web server
        //server.handleClient();
        
        // Check for data from serial tasks
        //if (xQueueReceive(roombaQueue, roombaBuffer, 0) == pdTRUE) {
            // Handle Roomba data (e.g., send to websocket clients)
            //handleRoombaNetwork(roombaBuffer);
        //}
        
       // if (xQueueReceive(lidarQueue, lidarBuffer, 0) == pdTRUE) {
            // Handle Lidar data
            //handleLidarNetwork(lidarBuffer);
       // }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
