#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/publisher.h>
#include <rosidl_runtime_c/string_functions.h>
#include "MPU9250.h"
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
// OTA Includes
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// DW1000 Configuration
char anchor_addr[] = "83:00:5B:D5:A9:9A:E2:8C"; //#4
uint16_t Adelay = 16527; 
float dist_m = 7.19; // meters

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

unsigned long startTime;

void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // WiFi and OTA Setup
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // OTA Configuration
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Anchor config and start");
  Serial.print("Antenna delay ");
  Serial.println(Adelay);
  Serial.print("Calibration distance ");
  Serial.println(dist_m);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // DW1000 Setup
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  startTime = millis();
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA updates
  DW1000Ranging.loop(); // Original ranging functionality
}

void newRange() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - startTime;
  
  Serial.print("Time elapsed: ");
  Serial.print(elapsedTime);
  Serial.print(" ms,");
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(", ");

  #define NUMBER_OF_DISTANCES 1
  float dist = 0.0;
  for (int i = 0; i < NUMBER_OF_DISTANCES; i++) {
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist = dist / NUMBER_OF_DISTANCES;
  Serial.print(dist);
  Serial.print("\abc");
  Serial.println();
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}