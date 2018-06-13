#include "Arduino.h"
#include "radio.h"
#include "quad_firmware.h"    // Header file with struct definition

// Initializing variables and structs for the orientation estimation data
data_t rdata;                             // Defining the empty struct to read the serial/binary data into:

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rfBegin(17);

  // Motor pins setup:
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
  // Motor and remote data
  uint8_t * input = (uint8_t*)&rdata;
  rfRead(input,sizeof(rdata));

  if(rdata.magic_number == m_number){
    analogWrite(motorpin1, rdata.throttle);
    analogWrite(motorpin2, rdata.throttle);
    analogWrite(motorpin3, rdata.throttle);
    analogWrite(motorpin4, rdata.throttle);
  }

}
