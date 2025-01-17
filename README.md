# Design Plan

ESP32-Wroom-32D has two CPU cores. Separate the Wifi and Micro-ROS comms on one core and Serial communication with Lidar and Roomba on another core.

# Notes:
1. ArduinoOTA fails most of the time but if you hold the boot button (right side) when OTA starts it works. need a better slution.