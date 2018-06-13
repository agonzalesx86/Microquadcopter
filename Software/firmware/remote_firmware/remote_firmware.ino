#include <radio.h>
#include <serLCD.h>
#include <quad_remote.h>     // Header file with pin definitions and setup
#include <quad_firmware.h>    // Header file with struct definition 
#include <serLCD.h> 

// Defining the struct to read the values into for the remote
data_t data;
parameters_t parameters;
serLCD lcd;

// initialize values
float throttle = 0;               
float yaw = 0;
float pitch = 0;
float roll = 0;
int pot1 = 0;
int pot2 = 0;
int button1 = 0;
int button2 = 0;
int menu;

int numbers[9] = {0,1,2,3,4,5,6,7, 0};


/*
const int numofscreens = 10;
String screens[numofscreens][2] = {{"Proportional gain","Kp"}, {"Integral gain","Ki"}, {"Derivative gain","Kd"}}
*/


// -------------------------------------------------------------------------------------------

void setup() 
{
  Serial.begin(115200);        // setup serial
  rfBegin(17);              // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)

  // Send a message to other RF boards on this channel
  //rfPrint("ATmega128RFA1 Dev Board Online!\r\n");

  pinMode(PIN_BTN1, INPUT_PULLUP);     // Button 1
  pinMode(PIN_BTN2, INPUT_PULLUP);     // Button 2

  delay(600); // delay needed before writing to the LCD Serial Display.

  
  //lcd.setBrightness(5);
  lcd.clear();
  lcd.clearLine(2);
  lcd.selectLine(2);
  lcd.print("Running...");
  lcd.selectLine(1);
  lcd.print("remote_firmware");
  delay(5000);
  lcd.clear();
  
}


void loop() 
{
  data.throttle = map(analogRead(analogTHROTTLE),149,816,0,255);
  data.yaw = map(analogRead(analogYAW),157,816,0,1024);
  data.pitch = map(analogRead(analogPITCH),158,816,0,1024);
  data.roll = map(analogRead(analogROLL),118,816,0,1024);

  data.pot1 = map(analogRead(analogPot1),110,820,0,3);
  data.pot2 = map(analogRead(analogPot2),110,820,0,3);

  data.button1 = digitalRead(analogButton1);
  data.button2 = digitalRead(analogButton2);
  data.magic_number = m_number;

  menu = data.pot1;
  data = LCD_Menu(menu,data);

  

  // Print to serial if press registered
  if (button1 == 0)
  {
    numbers[6] = 1024;
  } else {
    numbers[6] = 0;
  }

  if (button2 == 0)
  {
    numbers[7] = 1024;
  } else {
    numbers[7] = 0;
  }


  // Send the data out to the quad
  uint8_t * output = (uint8_t*)&data;
  rfWrite(output,sizeof(data));
  delay(150);  


  /*
  lcd.clear();
  //lcd.setCursor(0,0);
  dtostrf(data.pitch,4,0,p_str);
  dtostrf(data.Kp,1,3,kp_str);
  dtostrf(data.Kd,1,3,kd_str);
  sprintf(line1,"Kp:%s",kp_str);
  sprintf(line2,"Kd:%s",kd_str);
  lcd.print(line1);
  lcd.setCursor(1,0);
  lcd.print(line2);
  */
 
  /*
  lcd.setCursor(1,0);
  lcd.print("P: ");
  lcd.setCursor(15,2);
  lcd.print(data.pitch);

  lcd.selectLine(1);
  lcd.setCursor(0,0);
  lcd.print("Th: ");
  lcd.setCursor(15,1);
  lcd.print(data.throttle);
*/

  /*
  Serial.print(data.throttle);
  Serial.print("\t");
  Serial.print(data.yaw);
  Serial.print("\t");
  Serial.print(data.roll);
  Serial.print("\t");
  Serial.print(data.pitch);
  Serial.print("\t");
  */
  Serial.print(data.pot1);
  Serial.print("\t");
  Serial.print(data.pot2);
  Serial.print("\t");
  Serial.print(data.button1);
  Serial.print("\t");
  Serial.println(data.button2);
  //delay(200);
  
  
}                                                                                                                                                                                
