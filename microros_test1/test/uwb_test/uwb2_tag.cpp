#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h> // Include for string assignment
#include "MPU9250.h"


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

void setup()
{
    Serial.begin(115200);
    delay(1000);
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
}
 
void loop()
{
    DW1000Ranging.loop();
}
 // Variables to store previous data for comparison
float previous_range = -1.0;  // Initialize with a value that won't match any valid range
// unsigned long last_report_time = 0;
unsigned int range_count = 0;
  // Initialize time tracking variables
// Variables to store previous data for comparison
float previous_distance = -1.0;  // Initializing with a value that doesn't match any valid distance
unsigned long last_report_time = 0;
unsigned int message_count = 0;
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
}
 
void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}