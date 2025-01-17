#ifndef ESP32_Lidar_Roomba_Logger_H
#define ESP32_Lidar_Roomba_Logger_H
#include <HardwareSerial.h>

#define LOG_PRINT_LEVEL LOG_DEBUG // print at this level. 

typedef enum LogLevel {
    LOG_TRACE = 0,    // Most detailed information for tracing execution
    LOG_DEBUG = 1,    // Debugging information
    LOG_INFO = 2,     // General information about program execution
    LOG_WARN = 3,     // Warning messages for potentially harmful situations
    LOG_ERROR = 4,    // Error messages for serious problems
    LOG_FATAL = 5     // Critical errors that prevent program execution
} LogLevel;

void logPrint(LogLevel level, char* format, ...);

#endif // ESP32_Lidar_Roomba_Logger_H