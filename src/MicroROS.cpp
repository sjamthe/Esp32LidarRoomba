#include "esp32-hal.h"
#include <sys/_types.h>
#include <sys/_stdint.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

#include "Logger.h"
#include "Constants.h"

rcl_publisher_t scan_publisher;
std_msgs__msg__String scanMsg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

static bool ros_init_status = false;
static bool scan_publisher_status = false;
static unsigned long last_successful_publish = 0;
static unsigned long last_scan_publisher_create = 0;

extern QueueHandle_t lidarQueue;
extern bool lidar_status;
void handleLidar();
void initScanPublisher();

extern void processLidarData(char* buffer, int length); // TEMP bypassing LidarTask for now.
/* 
* Synchronize clock with the agent.
*/
void syncClock() {
	// Sync timeout
	const int timeout_ms = 1000;

	// Synchronize time with the agent
	rmw_uros_sync_session(timeout_ms);
	// After successful synchronization
	if (rmw_uros_epoch_synchronized()) {
		int64_t time_ns = rmw_uros_epoch_nanos();
		
		struct timeval tv;
		tv.tv_sec = time_ns / 1000000000LL;
		tv.tv_usec = (time_ns % 1000000000LL) / 1000;
		
		settimeofday(&tv, NULL);
	}
	struct tm timeinfo;
    char timeStringBuff[50];  // Make sure this is large enough for your format
	// Get and print local time
    if (getLocalTime(&timeinfo)) {
        strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S %Z", &timeinfo);
        logPrint(LOG_INFO, "Current time: %s", timeStringBuff);  
    } else {
        logPrint(LOG_ERROR, "Failed to obtain time");
    }
	logPrint(LOG_INFO, "Clock Synchronized with Micro-ROS agent");
}

void setupMicroROS() {
	static struct micro_ros_agent_locator locator;

	logPrint(LOG_INFO, "Starting MicroROS connecting to agent on IP %s, port %d",ROS_AGENT_IP, ROS_AGENT_PORT);
	locator.address.fromString(ROS_AGENT_IP);
    locator.port = ROS_AGENT_PORT;

	pinMode(LED_PIN, OUTPUT);
  	digitalWrite(LED_PIN, HIGH);

	// Set the agent IP and port
	if(rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        arduino_wifi_transport_open,
        arduino_wifi_transport_close,
        arduino_wifi_transport_write,
        arduino_wifi_transport_read
    ) != RMW_RET_OK) {
		logPrint(LOG_ERROR, "Error establishing micro-ROS wifi support.");
        ros_init_status = false;
		digitalWrite(LED_PIN, LOW);
        return;
	}
	logPrint(LOG_INFO, "Established support");


    // Get the default allocator
	allocator = rcl_get_default_allocator();
	if (!rcutils_allocator_is_valid(&allocator)) {
		logPrint(LOG_ERROR, "Error: Failed to get Micro-ROS allocator");
		digitalWrite(LED_PIN, LOW);
		return;
	}
	logPrint(LOG_INFO, "MicroROS got allocator");

	// Initialize micro-ROS support
	if (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) {
		rcl_ret_t retval = rclc_support_init(&support, 0, NULL, &allocator);
		if(retval != RCL_RET_OK) {
			logPrint(LOG_ERROR, "Error(%d) initializing micro-ROS support.", retval);
			ros_init_status = false;
			vTaskDelay(pdMS_TO_TICKS(1000));
			return;
		}
	} else {
		logPrint(LOG_ERROR, 
		"Agent not responding. Make sure  micro_ros_agent is running on IP %s, port %d",ROS_AGENT_IP, ROS_AGENT_PORT);
		ros_init_status = false;
		vTaskDelay(pdMS_TO_TICKS(1000));
		return;
	}
  	ros_init_status = true;
	logPrint(LOG_INFO, "MicroROS support started connecting to agent on IP %s, port %d",ROS_AGENT_IP, ROS_AGENT_PORT);

	syncClock();

	// Create node
	node = rcl_get_zero_initialized_node(); // get zero initialized node
	rcl_ret_t retval = rclc_node_init_default(&node, ROS_NODE_NAME, ROS_NAMESPACE, &support);
	if(retval != RCL_RET_OK) {
        logPrint(LOG_ERROR, "Error(%d) creating micro-ROS node. %s",retval, rcutils_get_error_string());
		rcl_reset_error();
        return;
	}
	logPrint(LOG_INFO, "MicroROS Node %s started", ROS_NODE_NAME);

	//create scan publisher
	initScanPublisher();

}

void cleanupMicroROS() {
	// Clean up micro-ROS 
	rcl_publisher_fini(&scan_publisher, &node);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

void initScanPublisher() {
	
	//zero initialization
	//memset(&scan_publisher, 0, sizeof(rcl_publisher_t));
	scan_publisher = rcl_get_zero_initialized_publisher();
	last_scan_publisher_create = millis();

	// get & set options, this ensures publisher doesn't return errors(1), just does best effort. we get 10qps
	rmw_qos_profile_t publisher_qos = rmw_qos_profile_default;
	publisher_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
	publisher_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

	rcl_ret_t retval = rclc_publisher_init(
		&scan_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    	"scan",
		&publisher_qos);
	if(retval != RCL_RET_OK) {
		logPrint(LOG_ERROR, "Error(%d) creating scan publisher. %s",retval, rcutils_get_error_string().str);
		scan_publisher_status = false;
		rcl_reset_error();
		return;
	}
	scan_publisher_status = true;
	logPrint(LOG_INFO, "MicroROS publishing /scan topic now.");
}

void publishScanMessage(char* buffer) {
	// TODO: With the new best effort policy we din't get errors even if ros-agent is down.
	// so need a new way to reestablish connection after agent comes back up. 
	if(!scan_publisher_status && (millis() - last_scan_publisher_create) > 50000) {
		initScanPublisher();
		vTaskDelay(pdMS_TO_TICKS(100)); // delay a bit else we will try ro call same function again.
	}
	if(!rcl_publisher_is_valid(&scan_publisher)) {
		//logPrint(LOG_ERROR, "Scan publisher is not valid");
		return; // This is needed as publisher takes 50 seconds to start.
	}

	scanMsg.data.data = buffer;
	scanMsg.data.size = strlen(scanMsg.data.data);
	scanMsg.data.capacity = strlen(scanMsg.data.data);
    
	rcl_ret_t retval = rcl_publish(&scan_publisher, &scanMsg, NULL);
	//With the new best effort policy we din't get errors even if ros-agent is down.
	if(retval == RCL_RET_OK) {
		last_successful_publish = millis();
	} else {
		static unsigned long gap = millis() - last_successful_publish;
		logPrint(LOG_ERROR, "Error(%d) publishing scan., gap=%u, %s",retval, gap, rcl_get_error_string());
		rcl_reset_error();
		return;
	}
}

void readLidarData() {
  char lidarBuffer[LIDAR_BUFFER_SIZE];

    if (lidar_status && xQueueReceive(lidarQueue, lidarBuffer, 0) == pdTRUE) {
        // Handle Lidar data
        publishScanMessage(lidarBuffer);
    }
}

unsigned long current_time = millis();
void handleMicroROS() {
	
  	if (!ros_init_status) {
		setupMicroROS();
		current_time = millis();
	}
	//every 10 seconds check micro-ros ping status as it is too expensive to do every loop.
	if((millis() - current_time) > 10000) {
		if(rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
			logPrint(LOG_ERROR, "MicroROS-agent not connected. Ping failed.");
			if(!ros_init_status) cleanupMicroROS();
			vTaskDelay(pdMS_TO_TICKS(1000));
			ros_init_status = false;
			return;
		}
		current_time = millis();
	}
	// only read data and publish if agent is connected. 
	readLidarData(); 
	
	digitalWrite(LED_PIN, !digitalRead(LED_PIN));
	rcl_ret_t retval = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}