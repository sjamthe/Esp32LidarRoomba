#include <cstddef>
#include <string>
#include <ctime>
#include <sys/time.h>

#include "Logger.h"

static const size_t BUFFER_SIZE = 512;
static char writeBuffer[BUFFER_SIZE];
timeval _timeval;

// This class implements a circular buffer of BUFFER_SIZE bytes
// This way we don't lose some history log that was written prior to connection.
class CircularLogBuffer {
private:
    char buffer[BUFFER_SIZE];
    size_t writePos = 0;
    bool bufferFull = false;

public:
    void write(const char* str) {
        size_t len = strlen(str);
        for (size_t i = 0; i < len; i++) {
            buffer[writePos] = str[i];
            writePos = (writePos + 1) % BUFFER_SIZE;
            if (writePos == 0) bufferFull = true;
        }
    }

public:
  void read(char* outBuffer) {
    size_t start;
    size_t length;
    
    if (bufferFull) {
        start = writePos;
        length = BUFFER_SIZE;
    } else {
        start = 0;
        length = writePos;
    }
    
    size_t j = 0;
    for (size_t i = 0; i < length && j < BUFFER_SIZE-1; i++) {
        size_t pos = (start + i) % BUFFER_SIZE;
        outBuffer[j++] = buffer[pos];
    }
    outBuffer[j] = '\0';
    //reset the buffer as we read it
    writePos = 0;
    bufferFull = false;
  }
};

// Global instance
CircularLogBuffer circularLogBuffer;


#define DEBUG_TRACE 0

char *logLevelString(LogLevel level) {
    switch (level) {
        case LOG_TRACE: return "TRACE:";
        case LOG_DEBUG: return "DEBUG:";
        case LOG_INFO: return "INFO:";
        case LOG_WARN: return "WARN:";
        case LOG_ERROR: return "ERROR:";
        case LOG_FATAL: return "FATAL:";
        default: return "UNKNOWN";
    }
}

void logPrint(LogLevel level, char* format, ...) {
    char buffer[BUFFER_SIZE];

    if (level < LOG_PRINT_LEVEL) {
        return;
    }

    va_list args;
    va_start(args, format);
    // Use vsnprintf for safe buffer handling
    int result = vsnprintf(buffer, BUFFER_SIZE-1, format, args);
    va_end(args);
    
    // Add timestamp
    //time_t now;
    //time(&now);
    gettimeofday(&_timeval, NULL); 
    struct tm timeinfo;
    localtime_r(&_timeval.tv_sec, &timeinfo);

    char timestamp[19];
    snprintf(timestamp, sizeof(timestamp), "[%02d:%02d:%02d.%06d] ",
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,_timeval.tv_usec );

    // If Serial is connected we write to it else to the circular log buffer.  
    // on ESP32 there is no guarrantee that someone is listening on other end of serial.  
    if (Serial && Serial.availableForWrite()) {
      // Flush circular buffer first.
      circularLogBuffer.read(writeBuffer);
      Serial.print(writeBuffer);

      Serial.print(timestamp);
      Serial.print(logLevelString(level));
      Serial.println(buffer);
    } else {
      circularLogBuffer.write(timestamp);
      circularLogBuffer.write(logLevelString(level));
      circularLogBuffer.write(buffer);
      circularLogBuffer.write("\n");
    }

}