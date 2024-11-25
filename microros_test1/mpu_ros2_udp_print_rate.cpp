#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include "MPU9250.h"
#include <rclc/publisher.h>

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
IPAddress agent_ip(192, 168, 2, 11);
size_t agent_port = 8888;
char ssid[] = "";
char psk[] = "";

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  // Initialize the MPU9250 sensor
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    while (1) {}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(9); // Set to 9 for a 100 Hz update rate

  // Initialize micro-ROS components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "mpu9250_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));

  // Create a timer with a 10 ms timeout (100 Hz)
  const unsigned int timer_timeout = 5;
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
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
