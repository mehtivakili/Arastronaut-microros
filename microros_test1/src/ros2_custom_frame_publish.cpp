#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h> // Include for string assignment

#include <SPI.h>
#include "DW1000Ranging.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin


void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();

// micro-ROS variables
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
    // Populate UWB message
    uwb_msg.range = DW1000Ranging.getDistantDevice()->getRange();
    uwb_msg.min_range = 0.0;
    uwb_msg.max_range = 100.0;
    uwb_msg.header.stamp.sec = millis() / 1000;
    uwb_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    RCSOFTCHECK(rcl_publish(&uwb_publisher, &uwb_msg, NULL));

    // Populate IMU message
    imu_msg.linear_acceleration.x = 0;
    imu_msg.linear_acceleration.y = 0;
    imu_msg.linear_acceleration.z = 0;
    imu_msg.angular_velocity.x = 0;
    imu_msg.angular_velocity.y = 0;
    imu_msg.angular_velocity.z = 0;
    imu_msg.orientation.x = 0;
    imu_msg.orientation.y = 0;
    imu_msg.orientation.z = 0;
    imu_msg.orientation.w = 0;
    imu_msg.header.stamp.sec = millis() / 1000;
    imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

    // Populate Magnetometer message
    mag_msg.magnetic_field.x = 0;
    mag_msg.magnetic_field.y = 0;
    mag_msg.magnetic_field.z = 0;
    mag_msg.header.stamp.sec = millis() / 1000;
    mag_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports("MobinNet-2.4G-9DA4", "aras5113", IPAddress(192, 168, 1, 141), 8888);
  delay(2000);

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

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "sensor_node", "", &support));

  // Initialize UWB publisher
  RCCHECK(rclc_publisher_init_default(
    &uwb_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "uwb_data"));

  // Initialize IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_data"));

  // Initialize Magnetometer publisher
  RCCHECK(rclc_publisher_init_default(
    &mag_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "mag_data"));

  const unsigned int timer_timeout = 100; // 100 ms
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  // Spin the executor to handle the timer callbacks
  DW1000Ranging.loop();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}
 
void newDevice(DW1000Device *device)
{
    Serial.print("ranging init; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}
 
void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}