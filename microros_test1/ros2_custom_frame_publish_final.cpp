#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h> // For string assignment
#include "MPU9250.h"

#include <SPI.h>
#include "DW1000Ranging.h"

// SPI Configuration
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// Shared Variables and Mutex for Thread Safety
float current_distance = -1.0; // Shared variable for UWB distance
float previous_distance = -1.0; // To store the last published distance
SemaphoreHandle_t distance_mutex; // Mutex to protect access to current_distance

// Connection Pins for DW1000
const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin
const uint8_t PIN_SS = 5;   // SPI Select pin
// Variables to store previous data for comparison
unsigned long last_report_time = 0;
unsigned int message_count = 0;
int ID;

// MPU9250 Initialization (Assuming I2C is used)
MPU9250 IMU(Wire, 0x68);

// Function Prototypes
void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();

// micro-ROS Variables
rcl_publisher_t uwb_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;

sensor_msgs__msg__Range uwb_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Define Macros for Error Handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error Handling Function
void error_loop() {
  while (1) {
    delay(100);
  }
}


// UWB Task Handle
TaskHandle_t UWBTaskHandle = NULL;

// Timer Callback Function (Runs on Core 0)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  if (timer != NULL) {
    // Acquire mutex to safely read current_distance
    if (xSemaphoreTake(distance_mutex, portMAX_DELAY)) {
      float distance = current_distance;
      xSemaphoreGive(distance_mutex);
      
      // Check if the current distance is different from the previous distance
      if (distance != previous_distance) {
        // Update previous distance
        previous_distance = distance;
        
        // Increment message count since a new unique message is published
        message_count++;
      }
      
      // Update UWB Message
      uwb_msg.range = distance;
      uwb_msg.min_range = ID; // Assuming ID represents min_range, adjust as needed
      uwb_msg.max_range = 100.0;
      
      // Timestamp for UWB
      unsigned long current_millis = millis();
      uwb_msg.header.stamp.sec = current_millis / 1000;
      uwb_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
      
      // Publish UWB Data
      RCSOFTCHECK(rcl_publish(&uwb_publisher, &uwb_msg, NULL));
    
      // --- IMU Data Handling ---
      IMU.readSensor(); // Non-blocking read
    
      // Populate IMU Message
      imu_msg.linear_acceleration.x = IMU.getAccelX_mss();
      imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
      imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();
    
      imu_msg.angular_velocity.x = IMU.getGyroX_rads();
      imu_msg.angular_velocity.y = IMU.getGyroY_rads();
      imu_msg.angular_velocity.z = IMU.getGyroZ_rads();
    
      // Optional: Populate orientation if available (e.g., from a filter)
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 1.0;
    
      // Timestamp for IMU
      imu_msg.header.stamp.sec = current_millis / 1000;
      imu_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
    
      // Publish IMU Data
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    
      // --- Magnetometer Data Handling ---
      mag_msg.magnetic_field.x = IMU.getMagX_uT();
      mag_msg.magnetic_field.y = IMU.getMagY_uT();
      mag_msg.magnetic_field.z = IMU.getMagZ_uT();
    
      // Timestamp for Magnetometer
      mag_msg.header.stamp.sec = current_millis / 1000;
      mag_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
    
      // Publish Magnetometer Data
      RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));

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
  }


// UWB Task Function (Runs on Core 1)
void UWB_Task(void * parameter) {
  while (1) {
    // Handle UWB ranging
    DW1000Ranging.loop();
    
    // Yield to other tasks
    vTaskDelay(1);
  }
}

// Callback Functions for DW1000Ranging
void newRange() {
  // Acquire mutex to safely update current_distance
  if (xSemaphoreTake(distance_mutex, portMAX_DELAY)) {
    current_distance = DW1000Ranging.getDistantDevice()->getRange();
    xSemaphoreGive(distance_mutex);
  }
  
  // Serial Print the new range
  // Serial.print("from: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  // Serial.print("\t Range: ");
  // Serial.print(current_distance);
  // Serial.print(" m");
  // Serial.print("\t RX power: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  // Serial.println(" dBm");
}

void newDevice(DW1000Device *device) {
  Serial.print("Ranging initialized; Device added! -> Short Address: ");
  Serial.println(device->getShortAddress(), HEX);
  ID = device->getShortAddress();
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Inactive device removed: Short Address: ");
  Serial.println(device->getShortAddress(), HEX);
}

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for Serial Port
  
  // Initialize Mutex
  distance_mutex = xSemaphoreCreateMutex();
  if (distance_mutex == NULL) {
    Serial.println("Failed to create mutex");
    while (1) { delay(100); }
  }

  // Initialize micro-ROS over Wi-Fi
  IPAddress agent_ip(192, 168, 2, 9);
  size_t agent_port = 8888;
  set_microros_wifi_transports("D-Link", "09124151339", agent_ip, agent_port);
  delay(2000); // Allow Time for Connection
  
  // Initialize UWB (DW1000Ranging)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pins
  
  // Attach Callbacks
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  
  // Start as Tag
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  Serial.println("UWB Initialized");
  
  // Initialize MPU9250 IMU
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    while (1) { delay(10); }
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(9); // Set Sample Rate Divider
  Serial.println("IMU Initialized");
  
  // Initialize micro-ROS Components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "sensor_node", "", &support));
  
  // Initialize UWB Publisher
  RCCHECK(rclc_publisher_init_default(
    &uwb_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "uwb/data_raw"));
  
  // Initialize IMU Publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));
  
  // Initialize Magnetometer Publisher
  RCCHECK(rclc_publisher_init_default(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag_raw"));
  
  // Initialize ROS Messages
  sensor_msgs__msg__Range__init(&uwb_msg);
  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__MagneticField__init(&mag_msg);
  
  // Initialize Frame IDs
  rosidl_runtime_c__String__init(&uwb_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&uwb_msg.header.frame_id, "uwb_link");
  
  rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  
  rosidl_runtime_c__String__init(&mag_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&mag_msg.header.frame_id, "imu_link");
  
  // Create a Timer with 10ms Interval (100Hz)
  const unsigned int timer_timeout = 10; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Initialize Executor and Add Timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // 2 handles for publishers
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Create UWB Task on Core 1
  xTaskCreatePinnedToCore(
    UWB_Task,          // Task function
    "UWB_Task",        // Name of the task
    4096,              // Stack size (bytes)
    NULL,              // Task input parameter
    1,                 // Priority of the task
    &UWBTaskHandle,    // Task handle
    1);                // Core where the task should run (Core 1)
  
  if (UWBTaskHandle == NULL) {
    Serial.println("Failed to create UWB Task");
    while (1) { delay(100); }
  }
}

void loop() {
  // Spin the executor to handle timer callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Optional: Add a small delay to prevent watchdog resets
  delay(1);
}
