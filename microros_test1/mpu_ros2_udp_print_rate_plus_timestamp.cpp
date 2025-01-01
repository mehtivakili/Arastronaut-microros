#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include "MPU9250.h"
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h> // Include for string assignment

// Create an MPU9250 object with the sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

// micro-ROS variables
rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Variables for monitoring publishing rate
unsigned long last_report_time = 0;
unsigned int message_count = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handling function
void error_loop() {
  while (1) {
    delay(100);
  }
}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read sensor data
    IMU.readSensor();

    // Populate the IMU message
    imu_msg.linear_acceleration.x = IMU.getAccelX_mss();
    imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
    imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();

    imu_msg.angular_velocity.x = IMU.getGyroX_rads();
    imu_msg.angular_velocity.y = IMU.getGyroY_rads();
    imu_msg.angular_velocity.z = IMU.getGyroZ_rads();

    // Optional: Populate orientation if available
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;

    // *** Begin Timestamp Addition ***
    // Retrieve the current time using micros()
    static unsigned long previous_micros = 0;
    unsigned long current_micros = micros();

    // Handle potential overflow of micros()
    if (current_micros < previous_micros) {
      // Overflow occurred
      previous_micros = current_micros;
    }

    unsigned long elapsed_micros = current_micros - previous_micros;
    previous_micros = current_micros;

    // Calculate total elapsed time since the program started
    // Note: micros() rolls over approximately every 70 minutes
    unsigned long total_micros = current_micros;

    unsigned long sec = total_micros / 1000000;
    unsigned long nsec = (total_micros % 1000000) * 1000;

    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nsec;
    // *** End Timestamp Addition ***

    // *** Begin Frame ID Assignment ***
    // Assign the frame_id only once to reduce overhead
    static bool frame_id_assigned = false;
    if (!frame_id_assigned) {
      // Initialize the string
      rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
      // Assign the desired frame ID
      rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
      frame_id_assigned = true;
    }
    // *** End Frame ID Assignment ***

    // Publish the IMU message
    RCSOFTCHECK(rcl_publish(&publisher, &imu_msg, NULL));
    message_count++;

    // Check if one second has passed to report the publishing rate
    unsigned long current_time = millis();
    if (current_time - last_report_time >= 1000) {
      Serial.print("Publishing rate: ");
      Serial.print(message_count);
      Serial.println(" Hz");
      message_count = 0;
      last_report_time = current_time;
    }
  }
}

// Wi-Fi configuration
IPAddress agent_ip(192, 168, 1, 141);
size_t agent_port = 8888;
char ssid[] = "MobinNet-2.4G-9DA4"; // Replace with your Wi-Fi SSID
char psk[] = "aras5113"; // Replace with your Wi-Fi password

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial port to be available

  // Initialize micro-ROS transports over Wi-Fi
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000); // Allow time for the connection to establish

  // Initialize the MPU9250 sensor
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    while (1) {}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(4); // Set to 9 for a 100 Hz update rate
  Serial.println("imu initialized");
  // Initialize micro-ROS components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "mpu9250_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));

  // Initialize the IMU message
  // It's important to initialize the message before using it
  sensor_msgs__msg__Imu__init(&imu_msg);

  // *** Begin Frame ID Initialization ***
  // Initialize the frame_id string
  rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
  // Assign the desired frame ID
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  // *** End Frame ID Initialization ***

  // Create a timer with a 10 ms timeout (100 Hz)
  const unsigned int timer_timeout = 5; // 5 ms for 200 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create an executor and add the timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Initialize time tracking variables
  last_report_time = millis();
  message_count = 0;
}

void loop() {
  // Spin the executor to handle the timer callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
