#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS 2 Message Types
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/string.h>

// ROS 2 Utilities
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h>

// Sensor Libraries
#include <SPI.h>
#include "DW1000Ranging.h"
#include "MPU9250.h"

// Madgwick Filter Library
#include "MadgwickAHRS.h"

// Pin Definitions
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

const uint8_t PIN_RST = 27; // Reset pin for DW1000
const uint8_t PIN_IRQ = 34; // IRQ pin for DW1000

// Wi-Fi Credentials and micro-ROS Agent IP
// const char WIFI_SSID[] = "D-Link";                   // Wi-Fi SSID
// const char WIFI_PASSWORD[] = "09124151339"; 

const char WIFI_SSID[] = "Infinite-NW";                   // Wi-Fi SSID
const char WIFI_PASSWORD[] = "00000000"; // Wi-Fi Password
const char TAG_ADDRESS[] = "7D:00:22:EA:82:60:3B:9C"; // UWB Tag Address

IPAddress AGENT_IP(192, 168, 2, 9); // Replace with your micro-ROS agent IP
const size_t AGENT_PORT = 8888;     // Replace with your micro-ROS agent port

// Shared Variables and Mutex for Thread Safety
float current_distance = -1.0;    // Shared variable for UWB distance
int ID = 0;                        // Current device ID
SemaphoreHandle_t distance_mutex;  // Mutex to protect access to current_distance and ID

// MPU9250 IMU Initialization
MPU9250 IMU(Wire, 0x68);

// Madgwick Filter Initialization
Madgwick filter;

// Function Prototypes
void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();
void publish_transform(float x, float y, float z, float qx, float qy, float qz, float qw, unsigned long current_millis);

// micro-ROS Variables
rcl_publisher_t uwb_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t transform_publisher;

sensor_msgs__msg__Range uwb_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
std_msgs__msg__String transform_msg;

// Initialize Executor and Support Structures
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error Handling Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void calibrate_magnetometer(float *data, const float offsets[3], const float scales[3][3], const float scale_factors[3]) {
    for (int i = 0; i < 3; ++i) {
        data[i] -= offsets[i]; // Hard iron correction
    }
    float corrected[3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            corrected[i] += scales[i][j] * data[j]; // Soft iron correction
        }
    }
    for (int i = 0; i < 3; ++i) {
        data[i] = corrected[i] / scale_factors[i]; // Scaling factors
    }
}

void apply_calibration(float *data, const float misalignment[3][3], const float scale[3][3], const float bias[3]) {
    float corrected[3] = {0};
    for (int i = 0; i < 3; ++i) {
        corrected[i] = data[i] - bias[i]; // Subtract bias
    }
    float aligned[3] = {0};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            aligned[i] += misalignment[i][j] * corrected[j]; // Apply misalignment matrix
        }
    }
    for (int i = 0; i < 3; ++i) {
        data[i] = aligned[i] * scale[i][i]; // Apply scale matrix
    }
}


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
    // Acquire mutex to safely read current_distance and ID
    if (xSemaphoreTake(distance_mutex, portMAX_DELAY)) {
      float distance = current_distance;
      int currentID = ID;
      xSemaphoreGive(distance_mutex);

       // Magnetometer Calibration Parameters
            float mag_offsets[3] = {-2.1702, 61.6808, -32.3492};
            float mag_scales[3][3] = {
                {0.9494, -0.0026, 0.0446},
                {-0.0026, 1.0267, 0.0145},
                {0.0446, 0.0145, 1.0380}};
            float mag_scale_factors[3] = {70.851, 69.54, 62.782};

            // Accelerometer Calibration Parameters
            float acc_misalignment[3][3] = {
                {1, -0.00014147, -4.84615e-06},
                {0, 1, -3.67728e-05},
                {-0, 0, 1}};
            float acc_scale[3][3] = {
                {0.998684, 0, 0},
                {0, 1.0007, 0},
                {0, 0, 0.989368}};
            float acc_bias[3] = {-0.178466, 0.0844203, 0.492449};

            // Gyroscope Calibration Parameters
            float gyro_misalignment[3][3] = {
                {1, -0.00164357, -0.0010404},
                {-0.000657191, 1, 0.0002104},
                {0.000465322, 0.00115319, 1}};
            float gyro_scale[3][3] = {
                {0.992201, 0, 0},
                {0, 1.01093, 0},
                {0, 0, 0.999627}};
            float gyro_bias[3] = {0.00167503, -0.000371569, 3.81379e-05};
      
      // Update UWB Message
      uwb_msg.range = distance;
      uwb_msg.min_range = 0.0;      // Define appropriately
      uwb_msg.max_range = 100.0;    // Define appropriately
      
      // Timestamp for UWB
      unsigned long current_millis = millis();
      uwb_msg.header.stamp.sec = current_millis / 1000;
      uwb_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
      
      // Assign frame_id (already set in setup)
      
      // Publish UWB Data
      RCSOFTCHECK(rcl_publish(&uwb_publisher, &uwb_msg, NULL));
    
      // --- IMU Data Handling ---
      IMU.readSensor(); // Non-blocking read
                float accel[3] = {IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss()};
            float gyro[3] = {IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads()};
            float mag[3] = {IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()};

            // Apply Calibration
            apply_calibration(accel, acc_misalignment, acc_scale, acc_bias);
            apply_calibration(gyro, gyro_misalignment, gyro_scale, gyro_bias);
            calibrate_magnetometer(mag, mag_offsets, mag_scales, mag_scale_factors);

      // Populate IMU Message
            imu_msg.linear_acceleration.x = accel[0];
            imu_msg.linear_acceleration.y = accel[1];
            imu_msg.linear_acceleration.z = accel[2];
            imu_msg.angular_velocity.x = gyro[0];
            imu_msg.angular_velocity.y = gyro[1];
            imu_msg.angular_velocity.z = gyro[2];

            mag_msg.magnetic_field.x = mag[0];
            mag_msg.magnetic_field.y = mag[1];
            mag_msg.magnetic_field.z = mag[2];
  
      // Populate orientation from Madgwick filter
      // Update filter with current sensor readings
      float ax = accel[0];
      float ay = accel[1];
      float az = accel[2];
      float gx = gyro[0] * 180.0f / PI; // Convert to degrees per second
      float gy = gyro[1] * 180.0f / PI; // Convert to degrees per second
      float gz = gyro[2] * 180.0f / PI; // Convert to degrees per second
      float mx = mag[0];
      float my = mag[1];
      float mz = mag[2];
  
      // Madgwick filter expects gyroscope data in degrees/sec
      filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  
      // Get the quaternion from the filter
      float q0 = filter.q0; // Accessible after making them public
      float q1 = filter.q1;
      float q2 = filter.q2;
      float q3 = filter.q3;
  
      // Populate IMU orientation
      imu_msg.orientation.x = q1;
      imu_msg.orientation.y = q2;
      imu_msg.orientation.z = q3;
      imu_msg.orientation.w = q0;
  
      // Timestamp for IMU
      imu_msg.header.stamp.sec = current_millis / 1000;
      imu_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
      
      // Assign frame_id (already set in setup)
  
      // Publish IMU Data
      RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  
      // --- Magnetometer Data Handling ---
      mag_msg.magnetic_field.x = mag[0];
      mag_msg.magnetic_field.y = mag[1];
      mag_msg.magnetic_field.z = mag[2];
  
      // Timestamp for Magnetometer
      mag_msg.header.stamp.sec = current_millis / 1000;
      mag_msg.header.stamp.nanosec = (current_millis % 1000) * 1000000;
      
      // Assign frame_id (already set in setup)
  
      // Publish Magnetometer Data
      RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
      
      // --- Publish Transform ---
      // Create a JSON string with transform data
      String json = "{";
      json += "\"parent_frame\": \"base_link\",";
      json += "\"child_frame\": \"imu_link\",";
      json += "\"translation\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.0},";
      json += "\"rotation\": {\"x\": " + String(q1, 6) + ", \"y\": " + String(q2, 6) + ", \"z\": " + String(q3, 6) + ", \"w\": " + String(q0, 6) + "}";
      json += "}";
      
      // Assign the JSON string to the std_msgs/String message
      rosidl_runtime_c__String__assign(&transform_msg.data, json.c_str());
      
      // Publish the Transform message on a custom topic
      RCSOFTCHECK(rcl_publish(&transform_publisher, &transform_msg, NULL));
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
  // Acquire mutex to safely update current_distance and ID
  if (xSemaphoreTake(distance_mutex, portMAX_DELAY)) {
    current_distance = DW1000Ranging.getDistantDevice()->getRange();
    ID = DW1000Ranging.getDistantDevice()->getShortAddress();
    xSemaphoreGive(distance_mutex);
  }
  
  // Optionally, print the new range for debugging
  /*
  Serial.print("from: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: ");
  Serial.print(current_distance);
  Serial.print(" m");
  Serial.print("\t RX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
  */
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
  
// Setup Function
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

    Serial.println("Uifi is connecting");

  // Initialize micro-ROS over Wi-Fi
    set_microros_wifi_transports("user", "pass", AGENT_IP, AGENT_PORT);

  // set_microros_wifi_transports("MobinNet-2.4G-9DA4", "aras5113", AGENT_IP, AGENT_PORT);
    Serial.println("Uifi ok");

  delay(2000); // Allow Time for Connection
  
  // Initialize UWB (DW1000Ranging)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, DW_CS, PIN_IRQ); // Reset, CS, IRQ pins
  
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
  
  // Initialize Madgwick Filter
  filter.begin(100); // Sample frequency in Hz (adjust as needed)
  
  // Initialize micro-ROS Components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "sensor_node", "", &support));
  
  // Initialize Publishers
  RCCHECK(rclc_publisher_init_best_effort(
    &uwb_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "uwb/data_raw"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data_raw"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "imu/mag_raw"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &transform_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "imu/transform"));
  
  // Initialize ROS Messages
  sensor_msgs__msg__Range__init(&uwb_msg);
  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__MagneticField__init(&mag_msg);
  std_msgs__msg__String__init(&transform_msg);
  
  // Initialize Frame IDs
  rosidl_runtime_c__String__assign(&uwb_msg.header.frame_id, "uwb_link");
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  rosidl_runtime_c__String__assign(&mag_msg.header.frame_id, "imu_link");
  
  // Create a Timer with 10ms Interval (100Hz)
  const unsigned int timer_timeout = 10; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Initialize Executor and Add Timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator)); // 1 handle for timer
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
  
  // Initialize Time Tracking Variables
  // (Optional, used for debugging or reporting)
}

// Loop Function
void loop() {
  // Spin the executor to handle timer callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
  // Optional: Add a small delay to prevent watchdog resets
  delay(1);
}