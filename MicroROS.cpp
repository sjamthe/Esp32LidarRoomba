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

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

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
  logPrint(LOG_INFO, "Starting MicroROS on IP %s, port %d\n",ROS_AGENT_IP, ROS_AGENT_PORT);
  // Set the agent IP and port
  set_microros_wifi_transport_only(ROS_AGENT_IP, ROS_AGENT_PORT);

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      logPrint(LOG_ERROR, "Error initializing micro-ROS support. %s \n",rcutils_get_error_string().str);
      return;
  }

  // Create node
  if(rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support)) {
      logPrint(LOG_ERROR, "Error creating node. %s \n",rcutils_get_error_string().str);
      return;
  }

  logPrint(LOG_INFO, "MicroROS started on IP %s, port %d\n",ROS_AGENT_IP, ROS_AGENT_PORT);
}