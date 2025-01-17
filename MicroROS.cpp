#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include "Logger.h"

#define ROS_AGENT_IP "192.168.86.36" // IP of machine running micro-ROS agent
#define ROS_AGENT_PORT 8888 // Port of machine running micro-ROS agent

#define ROS_NODE_NAME "esp32_node"
#define ROS_NAMESPACE ""

rcl_publisher_t scan_publisher;
std_msgs__msg__String scanMsg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

extern QueueHandle_t lidarQueue;
const int LIDAR_BUFFER_SIZE = 256;

static inline void set_microros_wifi_transport_only(char * agent_ip, uint agent_port) {
    static struct micro_ros_agent_locator locator;
    locator.address.fromString(agent_ip);
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        arduino_wifi_transport_open,
        arduino_wifi_transport_close,
        arduino_wifi_transport_write,
        arduino_wifi_transport_read
    );
}

void setupMicroROS() {
	logPrint(LOG_INFO, "Starting MicroROS on IP %s, port %d",ROS_AGENT_IP, ROS_AGENT_PORT);
	// Set the agent IP and port
	set_microros_wifi_transport_only(ROS_AGENT_IP, ROS_AGENT_PORT);

	allocator = rcl_get_default_allocator();

	// Initialize micro-ROS support
	if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        logPrint(LOG_ERROR, "Error initializing micro-ROS support. %s",rcutils_get_error_string().str);
        return;
	}
	logPrint(LOG_INFO, "MicroROS support started on IP %s, port %d",ROS_AGENT_IP, ROS_AGENT_PORT);

	// Create node
	if(rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support)) {
        logPrint(LOG_ERROR, "Error creating node. %s",rcutils_get_error_string().str);
        return;
	}
	logPrint(LOG_INFO, "MicroROS Node %s started", ROS_NODE_NAME);

    // Create publisher
    rcl_ret_t retval = rclc_publisher_init_default(&scan_publisher,
		&node,
    	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    	"scan");
	if(retval != RCL_RET_OK) {
		logPrint(LOG_ERROR, "Error creating scan publisher. %s",rcutils_get_error_string().str);
		return;
	}
	logPrint(LOG_INFO, "MicroROS publishing /scan topic now.");
}

void cleanupMicroROS() {
	// Clean up micro-ROS 
	rcl_publisher_fini(&scan_publisher, &node);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

void publishScanMessage(char* buffer) {
	scanMsg.data.data = buffer;
	scanMsg.data.size = strlen(scanMsg.data.data);
	scanMsg.data.capacity = strlen(scanMsg.data.data);
    
	rcl_publish(&scan_publisher, &scanMsg, NULL);
}

void handleLidar() {
  char lidarBuffer[LIDAR_BUFFER_SIZE];

    if (xQueueReceive(lidarQueue, lidarBuffer, 0) == pdTRUE) {
        // Handle Lidar data
        publishScanMessage(lidarBuffer);
    }
}

void handleMicroROS() {
    handleLidar();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}