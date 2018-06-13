/* QUAD_FIRMWARE.H header file
Created by: Alex Gonzales, Ben Phan
Description:

*/
#include <serLCD.h>

// Initializing constants and variables
// float PI = 3.14159265359;
float A_roll;
float A_pitch;
float A_yaw;
float G_roll;
float G_pitch;
float G_yaw;
int m_number = 6130;
int val = 0;

// Initializing pin values used in the quad firmware
int motorpin1 = 3;
int motorpin2 = 4;
int motorpin3 = 5;
int motorpin4 = 8;

// Initializing pin values used in the remote firmware

int analogTHROTTLE = 1;         // pin corresponding to remote throttle control
int analogYAW = 0;              // pin corresponding to remote yaw control
int analogPITCH = 3;            // pin corresponding to remote pitch control
int analogROLL = 2;             // pin corresponding to remote roll control

int analogPot1 = 7;
int analogPot2 = 6;

int analogButton1 = 16;
int analogButton2 = 17;

// Defining the struct type to be used in both the quad and remote firmware
typedef struct {
  int throttle;
  float roll;
  float pitch;
  float yaw;
  int pot1;
  int pot2;
  int button1;
  int button2;
  float Kp;
  float Kd;
  float Ki;
  float C_gain;
  int magic_number;
} data_t;

//
	char line1[16];
	char line2[16];
	char state[16];
	char Kp_str[16];
	char Kd_str[16];
	char Ki_str[16];
	char C_str[16];

// Defining the struct type for the LCD Menu parameters
typedef struct {
   char state;
   float Kp;
   float Kd;
   float Ki;
   float C_gain;
} parameters_t;

// Calibration values for the quadcopter
float calibration_pitch_angle = 14.05;
int calibration_roll_angle = 0;
int calibration_yaw_angle = 0;

// Defining the struct type to be used for the filtered accelerometer and gyro readings.
typedef struct {
	float roll;
	float pitch;
	float yaw;
} orientationdata_t;

/*
// Defining the function to calculate the euler angles based on the linear accelerometers.
float get_euler(float x, float y){
	float angle;
	angle = atan2(y,x)*(180/3.14159265359);
	return angle;
}*/

float clamp(float var, float a, float b){
	if(var <= b && var >= a){
      var = var;
    }
    else if(var < a){
      var = a;
    }
    else{
      var = b;
    }
    return var;
}

// Defining the function to calculate the euler angles by integrating the gyro readings.
// Note: The approximation method used for the integral is Newton's method.
float integrate_gyro(float gyro_dot, float gyro_last, float DT){
	float gyro_angle;
	gyro_angle = gyro_last + gyro_dot*DT;
	return gyro_angle;
}

float integrate_error(float de, float error_last, float DT){
	float int_error;
	int_error = error_last + de*DT;
	return int_error;
}

float ButtonManager(float input, int increment, int button1, int button2){

	switch(increment){
		case 0:
		if(button2 == 0){
			input = input + 0.001;
		}
		
		else if(button1 == 0){
			
			input = input - 0.001;
		}
		 
		break;

		case 1:
		if(button2 == 0){
			input = input + 0.01;
		}
		else if(button1 == 0){
			input = input - 0.01;
		}
		break;

		case 2:
		if(button2 == 0){
			input = input + 0.1;
		}
		else if(button1 == 0){
			input = input - 0.1;
		}
		break;
	}
	
	return input;
} 


data_t LCD_Menu(int menu,data_t data){
	serLCD lcd;
	lcd.clear();

	switch(menu){
		case 0:
		data.Kp = ButtonManager(data.Kp, data.pot2, data.button1, data.button2);
		//data.Kp = data.Kp + 0.001;
		dtostrf(data.Kp,1,3,Kp_str);	
		sprintf(line1,"Kp: %s",Kp_str);
		break;

		case 1:
		data.Kd = ButtonManager(data.Kd,data.pot2,data.button1,data.button2);
		dtostrf(data.Kd,1,3,Kd_str);
		sprintf(line1,"Kd: %s",Kd_str);
		break;

		case 2:
		data.Ki = ButtonManager(data.Ki,data.pot2,data.button1,data.button2);
		dtostrf(data.Ki,1,3,Ki_str);
		//dtostrf(data.C_gain,1,3,C_str);
		sprintf(line1,"Ki: %s",Ki_str);
		//sprintf(line2,"C_gain: %s",C_str);
		break;
	}
	lcd.print(line1);
	lcd.setCursor(1,0);
	//lcd.print(line2);
	return data;
}


