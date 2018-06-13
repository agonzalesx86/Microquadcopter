 #include "Arduino.h"
#include "radio.h"
#include "quad_firmware.h"    // Header file with struct definition

// Libraries included for the IMU readings
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_Simple_AHRS.h>

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
// Create simple AHRS algorithm using the LSM9DS1 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

// Initializing variables and structs for the orientation estimation data
data_t rdata;                             // Defining the empty struct to read the serial/binary data into:
orientationdata_t orientation_f;
orientationdata_t accel;
orientationdata_t gyro;
float comp_gain = 0.950;
float gyro_last = 0;
float t_now;
float t_last = 0;
float delta_t;
float orientation_roll_last = 0;
float orientation_pitch_last = 0;

// Initialization of the variables for PID control
float error_now = 0;
float error_last = 0;
int range = 180;
int center = range/2;

const int e_index = 18; // number of errors to average.
float derivative_error[e_index];
int read_index = 0;
float total = 0;
int count;
int now20;
int last20 = millis();
float set_pitch;
float pitch;
float error;
float average_de;
float integral_error;
float integral_error1;
float integral_error2;
float PID_output;
float throttle;

void setupSensor()
{
  // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_119 | G_BW_G_10 );  //952hz ODR + 63Hz cuttof

  // Enable the XL (Section 7.23)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);

  // Set low-pass XL filter frequency divider (Section 7.25)
  lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_100);
  
  // enable mag continuous (Section 8.7)
  lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000); // continuous mode

  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
    while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("quad_firmware now running...");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring or I2c ADDR!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();
  
  
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

int imucount;

unsigned int last = millis();
void loop() {

    int now = millis();
    
    // Motor and remote data
    uint8_t * input = (uint8_t*)&rdata;
    rfRead(input,sizeof(rdata));

    
    // 0.3 ms latency -----------------------------------------------------------------------------------------------------
    // IMU data
    lsm.read();
    sensors_event_t a, m, g, temp;
    sensors_vec_t orientation;

    
    // 0.4 ms latency -----------------------------------------------------------------------------------------------------
    lsm.getEvent(&a, &m, &g, &temp);

    
    // 0.3 ms latency -----------------------------------------------------------------------------------------------------
    // if (ahrs.getOrientation(&orientation)){
    ahrs.getOrientation(&orientation);

    
    // 0.1 ms latency -----------------------------------------------------------------------------------------------------
    // Integrating the gyro data for filtering
    delta_t = (now - last)/1000;
    G_pitch = gyro_last + g.gyro.y*delta_t;
    gyro_last = g.gyro.y;

    // Complimentary Filter
    orientation_f.pitch = comp_gain*(orientation_pitch_last + delta_t*G_pitch) + (1 - comp_gain)*(orientation.pitch);
    orientation_pitch_last = orientation_f.pitch;

    
    
    // Error Calculations
    set_pitch = map(rdata.pitch,0,1023,0,range);
    pitch = orientation_f.pitch + center + calibration_pitch_angle;

    // Proportional Action
    error = set_pitch - pitch;
    error_now = error;

    
    // Derivative Action
    total = total - derivative_error[read_index];
    derivative_error[read_index] = 1000*(error_now - error_last)/(now-last);
    total = total + derivative_error[read_index];
    read_index = read_index + 1;

    if (read_index >= e_index){
      read_index = 0;
    }
    
    average_de = total/e_index;
    error_last = error_now;

    // Integral Action
    float decay_const = 0.90;
    integral_error1 = integrate_error(average_de, error_last, delta_t);
    integral_error2 = decay_const*integral_error2 + error;

    // PID Output
    PID_output = rdata.Kp*error + rdata.Kd*average_de + rdata.Ki*integral_error2;

    float duty1 = rdata.throttle + PID_output;
    float duty2 = rdata.throttle - PID_output;
    duty1 = clamp(duty1,0,255);
    duty2 = clamp(duty2,0,255);
    

    // Assigning the PWM values to the motors
    if(rdata.magic_number == m_number){
            
      analogWrite(motorpin1, duty2);
      analogWrite(motorpin2, duty2);
      analogWrite(motorpin3, duty1);
      analogWrite(motorpin4, duty1);
      
    }

    else{
      Serial.println("Bad Packet received!");
    }
    

    /*
    Serial.print(F(" "));
    //Serial.print(pitch);
    Serial.print(F(" "));
    //Serial.print(set_pitch);
    Serial.print(F(" "));
    Serial.print(error);
    Serial.print(F(" "));
    Serial.print(average_de);
    Serial.print(F(" "));
    //Serial.print(derivative_error_old);
    Serial.print(F(" "));
    //Serial.print(0);
    //Serial.print(derivative_input);
    //Serial.print(derivative_error);
    Serial.println(F("")); 
    */

    /*    
    if (count == 10){
      now20 = millis();
      //Serial.print(rdata.throttle);
      Serial.print(error);
      Serial.print(F(" "));
      Serial.print(integral_error1);
      Serial.print(F(" "));
      Serial.print(integral_error2);
      Serial.println(F(" "));
      //Serial.println(now20 - last20);
      //Serial.print("   ");
      //Serial.print(PID_output);
      //Serial.print("   ");
      //Serial.println(throttle);
      count = 0;
      last20 = now20;
      }
      else{
        count = count + 1;
      }*/
      
     
    
    
  //}

  
  
  last = now;
  

}
