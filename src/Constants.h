// File to store global constants
#define LED_PIN 2

#define ROS_AGENT_IP "192.168.86.36" // IP of machine running micro-ROS agent, TODO: Ad webapi to set this.
#define ROS_AGENT_PORT 8888 // Port of machine running micro-ROS agent. TODO: Ad webapi to set this.
#define ROS_NODE_NAME "esp32_lidar_roomba_node"
#define ROS_NAMESPACE ""

#define LIDAR_BUFFER_SIZE   256
#define LIDAR_SERIAL Serial2
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_MOTOR_PIN 5
