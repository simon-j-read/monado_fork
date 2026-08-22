/*
  Copyright (c) 2014-2015 NicoHood
  See the readme for credit to other people.

  Advanced RawHID example

  Shows how to send bytes via RawHID.
  Press a button to send some example values.

  Every received data is mirrored to the host via Serial.

  See HID Project documentation for more information.
  https://github.com/NicoHood/HID/wiki/RawHID-API
*/

#include "HID-Project.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"

Adafruit_ICM20948 icm;

const int pinLed = LED_BUILTIN;
const int pinButton = 2;

// Buffer to hold RawHID data.
// If host tries to send more data than this,
// it will respond with an error.
// If the data is not read until the host sends the next data
// it will also respond with an error and the data will be lost.
uint8_t rawhidData[255];

// Data packet that will be sent over HID to monado service
// Will be populated with the adafruit sensor_event_t values,
// and a timestamp.
struct TelemetryPacket {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  unsigned long timestamp;  // time since program started.
  uint8_t padding[8];
} __attribute__((packed));

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

TelemetryPacket packet;

void setup() {
  pinMode(pinLed, OUTPUT);
  pinMode(pinButton, INPUT_PULLUP);

  Serial.begin(115200);

  // Set the RawHID OUT report array.
  // Feature reports are also (parallel) possible, see the other example for this.
  RawHID.begin(rawhidData, sizeof(rawhidData));

   // Try to initialize!
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
}

void loop() {
  // Send data to the host

  digitalWrite(pinLed, HIGH);

  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Assign sensor event values to HID packet.

  packet.accelX = accel.acceleration.x;
  packet.accelY = accel.acceleration.y;
  packet.accelZ = accel.acceleration.z;
  packet.gyroX = gyro.gyro.x;
  packet.gyroY = gyro.gyro.y;
  packet.gyroZ = gyro.gyro.z;
  packet.timestamp = millis();

  // Create buffer with numbers and send it
  // uint8_t megabuff[100];
  // for (uint8_t i = 0; i < sizeof(megabuff); i++) {
  //   megabuff[i] = i;
  // }

  uint8_t megabuff[sizeof(packet)];
  memcpy(megabuff, &packet, sizeof(packet));

  RawHID.write(megabuff, sizeof(megabuff));
  digitalWrite(pinLed, LOW);
  // Simple debounce
  // Serial.println("looped");
  // delay(300);


  
}

