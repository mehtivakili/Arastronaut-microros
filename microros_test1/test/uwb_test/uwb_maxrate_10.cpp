#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h> // Include for string assignment
#include "MPU9250.h"

#include <SPI.h>
#include "DW1000Ranging.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5
 int ID;
// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

 
void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();

// Variables to store previous data for comparison
float previous_distance = -1.0;  // Initializing with a value that doesn't match any valid distance
unsigned long last_report_time = 0;
unsigned int message_count = 0;

// micro-ROS variables
rcl_publisher_t publisher;
sensor_msgs__msg__Range uwb_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;



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
    // DW1000Ranging.loop();
    float current_distance = DW1000Ranging.getDistantDevice()->getRange();
    uwb_msg.range = DW1000Ranging.getDistantDevice()->getRange();
    uwb_msg.max_range = ID;
    // Serial.println(uwb_msg.range);

    // Populate the IMU message
    // imu_msg.linear_acceleration.x = IMU.getAccelX_mss();
    // imu_msg.linear_acceleration.y = IMU.getAccelY_mss();
    // imu_msg.linear_acceleration.z = IMU.getAccelZ_mss();

    // imu_msg.angular_velocity.x = IMU.getGyroX_rads();
    // imu_msg.angular_velocity.y = IMU.getGyroY_rads();
    // imu_msg.angular_velocity.z = IMU.getGyroZ_rads();

    // // Optional: Populate orientation if available
    // imu_msg.orientation.x = 0.0;
    // imu_msg.orientation.y = 0.0;
    // imu_msg.orientation.z = 0.0;
    // imu_msg.orientation.w = 1.0;
    // Check if the distance has changed from the previous value
    if (current_distance != previous_distance) {
      // Update the previous distance with the current one
      previous_distance = current_distance;
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

    uwb_msg.header.stamp.sec = sec;
    uwb_msg.header.stamp.nanosec = nsec;
    // *** End Timestamp Addition ***

    // *** Begin Frame ID Assignment ***
    // Assign the frame_id only once to reduce overhead
    static bool frame_id_assigned = false;
    if (!frame_id_assigned) {
      // Initialize the string
      rosidl_runtime_c__String__init(&uwb_msg.header.frame_id);
      // Assign the desired frame ID
      rosidl_runtime_c__String__assign(&uwb_msg.header.frame_id, "imu_link");
      frame_id_assigned = true;
    }
    // *** End Frame ID Assignment ***

    // Publish the IMU message
    RCSOFTCHECK(rcl_publish(&publisher, &uwb_msg, NULL));
    message_count++;

    }

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
IPAddress agent_ip(192, 168, 2, 9);
size_t agent_port = 8888;
// char ssid[] = "MobinNet-2.4G-9DA4"; // Replace with your Wi-Fi SSID
// char psk[] = "aras5113"; // Replace with your Wi-Fi password
char ssid[] = "D-Link"; // Replace with your Wi-Fi SSID
char psk[] = "09124151339"; // Replace with your Wi-Fi password

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for serial port to be available

  // Initialize micro-ROS transports over Wi-Fi
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000); // Allow time for the connection to establish

    //init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    //define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    //Enable the filter to smooth the distance
    //DW1000Ranging.useRangeFilter(true);
 
    //we start the module as a tag
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  Serial.println("uwb initialized");

  // Initialize micro-ROS components
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "dw1000_node", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "uwb/data_raw"));

  // Initialize the IMU message
  // It's important to initialize the message before using it
  sensor_msgs__msg__Range__init(&uwb_msg);

  // *** Begin Frame ID Initialization ***
  // Initialize the frame_id string
  rosidl_runtime_c__String__init(&uwb_msg.header.frame_id);
  // Assign the desired frame ID
  rosidl_runtime_c__String__assign(&uwb_msg.header.frame_id, "imu_link");
  // *** End Frame ID Initialization ***

  // Create a timer with a 10 ms timeout (100 Hz)
  const unsigned int timer_timeout = 10; // 5 ms for 200 Hz
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
  DW1000Ranging.loop();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// void newRange()
// {
//     Serial.print("from: ");
//     Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
//     Serial.print("\t Range: ");
//     Serial.print(DW1000Ranging.getDistantDevice()->getRange());
//     Serial.print(" m");
//     Serial.print("\t RX power: ");
//     Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
//     Serial.println(" dBm");
// }

// Variables to store previous data for comparison
float previous_range = -1.0;  // Initialize with a value that won't match any valid range
// unsigned long last_report_time = 0;
unsigned int range_count = 0;

// Modified newRange function
void newRange() {
    // Get the new range from the DW1000 device
    float current_range = DW1000Ranging.getDistantDevice()->getRange();

    // Check if the range value has changed
    // if (current_range != previous_range) {
    if (true) {

        // Update previous range with the new value
        previous_range = current_range;

        // Print the new data
        Serial.print("from: ");
        Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
        Serial.print("\t Range: ");
        Serial.print(current_range);
        Serial.print(" m");
        Serial.print("\t RX power: ");
        Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
        Serial.println(" dBm");

        // Increment the range count
        range_count++;
    }

    // Monitor the printing rate per second
    unsigned long current_time = millis();
    if (current_time - last_report_time >= 1000) {  // Report every second
        Serial.print("Range updates per second: ");
        Serial.println(range_count);
        last_report_time = current_time;
        range_count = 0;  // Reset the count for the next second
    }
}

 
void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
    ID = device->getShortAddress();
}
 
void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
