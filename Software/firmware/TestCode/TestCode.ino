#include "Arduino.h"
#include "radio.h"
#include "quad_firmware.h"    // Header file with struct definition
#include <Servo.h>

// Libraries included for the IMU readings
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_Simple_AHRS.h>

int throttle = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("quad_firmware now running...");

  pinMode(motorpin1, OUTPUT);
  pinMode(motorpin2, OUTPUT);
  pinMode(motorpin3, OUTPUT);
  pinMode(motorpin4, OUTPUT);

  // Set the motors off first:
  analogWrite(motorpin1, 0);
  analogWrite(motorpin2, 0);
  analogWrite(motorpin3, 0);
  analogWrite(motorpin4, 0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  throttle = 0;
  analogWrite(3, throttle);
  analogWrite(4, throttle);
  analogWrite(5, throttle);
  analogWrite(8, throttle);
  
  /*
  analogWrite(motorpin2, throttle);
  analogWrite(motorpin3, throttle);
  analogWrite(motorpin4, throttle);
  */
}
