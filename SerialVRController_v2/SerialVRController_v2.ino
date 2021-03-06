#define LEFT

#include "freeram.h"

#include "mpu.h"
#include "I2Cdev.h"

#include "quaternionFilters.h"
#include "MPU9250.h"

// Button pinout
#define MENU_BUTTON_PIN 9
#define SYSTEM_BUTTON_PIN 7
#define GRIP_BUTTON_PIN 4
#define TRIGGER_BUTTON_PIN 2
#define TOUCHPAD_PRESS_PIN 3
// Make the controller layouts mirrored
#ifdef LEFT 
  #define POWER_BUTTON_PIN 5
  #define AT_BUTTON_PIN 10
  #define B_BUTTON_PIN 6
#else
  #define POWER_BUTTON_PIN 10
  #define AT_BUTTON_PIN 5
  #define B_BUTTON_PIN 8
#endif

#ifdef LEFT
  // The analog sticks are swapped on the left controller by mistake.
  #define ANALOG_X_PIN A7
  #define ANALOG_Y_PIN A6
#else
  #define ANALOG_X_PIN A6
  #define ANALOG_Y_PIN A7
#endif

#ifdef LEFT
  // Analog correction values for L controller
  #define ANALOG_DEAD 10
  #define ANALOG_X_CENTER 522
  #define ANALOG_X_MIN 0
  #define ANALOG_X_MAX 960
  #define ANALOG_Y_CENTER 514
  #define ANALOG_Y_MIN 50
  #define ANALOG_Y_MAX 1000
#else
  // Analog correction values for R controller
  #define ANALOG_DEAD 10
  #define ANALOG_X_CENTER 494
  #define ANALOG_X_MIN 0
  #define ANALOG_X_MAX 1023
  #define ANALOG_Y_CENTER 437
  #define ANALOG_Y_MIN 0
  #define ANALOG_Y_MAX 500
#endif

// Analog handling
int x_pos_range = 0;
int x_neg_range = 0;
int y_pos_range = 0;
int y_neg_range = 0; 

// Current button states
bool menuButtonDown = false;
bool systemButtonDown = false;
bool gripButtonDown = false;
bool triggerButtonDown = false;
bool touchpadDown = false;
bool atButtonDown = false;
int buttonStates = 0;
int lastButtonStates = 0;
float touchpadX = 0;
float touchpadY = 0;
bool powerButtonDown = false;
bool bButtonDown = false;

#ifdef LEFT
  char handLetter = 'L';
#else
  char handLetter = 'R';
#endif

#define ReadableSerial false

int ret;
void setup() {
   // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ANALOG_X_PIN, INPUT);
  pinMode(ANALOG_Y_PIN, INPUT);
  
  pinMode(MENU_BUTTON_PIN, INPUT);
  digitalWrite(MENU_BUTTON_PIN, HIGH);
  
  pinMode(SYSTEM_BUTTON_PIN, INPUT);
  digitalWrite(SYSTEM_BUTTON_PIN, HIGH);
  
  pinMode(GRIP_BUTTON_PIN, INPUT);
  digitalWrite(GRIP_BUTTON_PIN, HIGH);
  
  pinMode(TRIGGER_BUTTON_PIN, INPUT);
  digitalWrite(TRIGGER_BUTTON_PIN, HIGH);

  pinMode(TOUCHPAD_PRESS_PIN, INPUT);
  digitalWrite(TOUCHPAD_PRESS_PIN, HIGH);

  pinMode(POWER_BUTTON_PIN, INPUT);
  digitalWrite(POWER_BUTTON_PIN, HIGH);

  pinMode(AT_BUTTON_PIN, INPUT);
  digitalWrite(AT_BUTTON_PIN, HIGH);

  pinMode(B_BUTTON_PIN, INPUT);
  digitalWrite(B_BUTTON_PIN, HIGH);
  
  // Calculate initial analog values
  x_pos_range = ANALOG_X_MAX - ANALOG_X_CENTER;
  x_neg_range = ANALOG_X_MIN - ANALOG_X_CENTER;
  y_pos_range = ANALOG_Y_MAX - ANALOG_Y_CENTER;
  y_neg_range = ANALOG_Y_MIN - ANALOG_Y_CENTER;
  
  Fastwire::setup(400,0);
  Serial.begin(115200);
  ret = mympu_open(200);
  Serial.print("MPU init: "); Serial.println(ret);
  Serial.print("Free mem: "); Serial.println(freeRam());
	
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

float lastTime = 0;
float currentTime = 0;
void loop() {
    ret = mympu_update();

    switch (ret) {
    	case 0: c++; break;
    	case 1: np++; return;
    	case 2: err_o++; return;
    	case 3: err_c++; return; 
    	default: 
    		Serial.print("READ ERROR!  ");
    		Serial.println(ret);
    		return;
    }

    ReadControllerData();
    SendDataOnSerial();
    //PrintAnalogValues();
}

void SendSensorDataProcessing() {    
  Serial.print("&,");
  Serial.print(mympu.quat.w, 8);
  Serial.print(",");
  Serial.print(mympu.quat.x, 8);
  Serial.print(",");
  Serial.print(mympu.quat.y, 8);
  Serial.print(",");
  Serial.print(mympu.quat.z, 8);

  Serial.print(",");
  Serial.print(mympu.gyro[0],8);
  Serial.print(",");
  Serial.print(mympu.gyro[1],8);
  Serial.print(",");
  Serial.print(mympu.gyro[2],8);

  Serial.print(",");
  Serial.print(mympu.ypr[0],8);
  Serial.print(",");
  Serial.print(mympu.ypr[1],8);
  Serial.print(",");
  Serial.print(mympu.ypr[2],8);
  
  
  Serial.print('|');
}

void PrintAnalogValues()
{
  float an_x = analogRead(ANALOG_X_PIN);
  float an_y = analogRead(ANALOG_Y_PIN);
  Serial.print("X: ");
  Serial.print(an_x);
  Serial.print(" Y: ");
  Serial.println(an_y);
}

void SendDataOnSerial()
{  
  Serial.print(handLetter);
  Serial.print(";");
  Serial.print(buttonStates);
  Serial.print(";");
  Serial.print(touchpadX);
  Serial.print(";");
  Serial.print(touchpadY);
  Serial.print(";");
  Serial.print(mympu.quat.w, 4);
  Serial.print(";");
  Serial.print(mympu.quat.x, 4);
  Serial.print(";");
  Serial.print(mympu.quat.y, 4);
  Serial.print(";");
  Serial.print(mympu.quat.z, 4);
  
  Serial.print('|');
}

void ReadControllerData()
{ 
  // Menu button
  if (!menuButtonDown && digitalRead(MENU_BUTTON_PIN) == LOW)
  {
    // Menu button pressed
    menuButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Menu button down");
    buttonStates |= 4;
  }
  else if (menuButtonDown && digitalRead(MENU_BUTTON_PIN) == HIGH)
  {
    // Menu button released
    digitalWrite(LED_BUILTIN, LOW);
    menuButtonDown = false;
    buttonStates &= B11011;
  }

  // System button
  if (!systemButtonDown && digitalRead(SYSTEM_BUTTON_PIN) == LOW)
  {
    // System button pressed
    systemButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("System button down");
    buttonStates |= 2;
  }
  else if (systemButtonDown && digitalRead(SYSTEM_BUTTON_PIN) == HIGH)
  {
    // System button released
    digitalWrite(LED_BUILTIN, LOW);
    systemButtonDown = false;
    buttonStates &= B11101;
  }

  // Grip button
  if (!gripButtonDown && digitalRead(GRIP_BUTTON_PIN) == LOW)
  {
    // Grip button pressed
    gripButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Grip button down");
    buttonStates |= 1;
  }
  else if (gripButtonDown && digitalRead(GRIP_BUTTON_PIN) == HIGH)
  {
    // Grip button released
    digitalWrite(LED_BUILTIN, LOW);
    gripButtonDown = false;
    buttonStates &= B11110;
  }

  // Trigger button
  if (!triggerButtonDown && digitalRead(TRIGGER_BUTTON_PIN) == LOW)
  {
    // Trigger button pressed
    triggerButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Trigger button down");
    buttonStates |= 16;
  }
  else if (triggerButtonDown && digitalRead(TRIGGER_BUTTON_PIN) == HIGH)
  {
    // Trigger button released
    digitalWrite(LED_BUILTIN, LOW);
    triggerButtonDown = false;
    buttonStates &= B01111;
  }

  // Touchpad press
  if (!touchpadDown && digitalRead(TOUCHPAD_PRESS_PIN) == LOW)
  {
    // Touchpad pressed
    touchpadDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    //Serial.print("Touchpad press down");
    buttonStates |= 8;
  }
  else if (touchpadDown && digitalRead(TOUCHPAD_PRESS_PIN) == HIGH)
  {
    // Touchpad released
    digitalWrite(LED_BUILTIN, LOW);
    touchpadDown = false;
    buttonStates &= B10111;
  }

  // Power button press
  if (!powerButtonDown && digitalRead(POWER_BUTTON_PIN) == LOW)
  {
    // Power button pressed
    powerButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    buttonStates |= 32;
  }
  else if (powerButtonDown && digitalRead(POWER_BUTTON_PIN) == HIGH)
  {
    // Power button released
    digitalWrite(LED_BUILTIN, LOW);
    powerButtonDown = false;
    buttonStates &= B011111;
  }

  // At (@) button press
  if (!atButtonDown && digitalRead(AT_BUTTON_PIN) == LOW)
  {
    // At button pressed
    atButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    buttonStates |= 64;
  }
  else if (atButtonDown && digitalRead(AT_BUTTON_PIN) == HIGH)
  {
    // At button released
    digitalWrite(LED_BUILTIN, LOW);
    atButtonDown = false;
    buttonStates &= B0111111;
  }

  // B button press
  if (!bButtonDown && digitalRead(B_BUTTON_PIN) == LOW)
  {
    // B button pressed
    bButtonDown = true;
    digitalWrite(LED_BUILTIN, HIGH);
    buttonStates |= 128;
  }
  else if (bButtonDown && digitalRead(B_BUTTON_PIN) == HIGH)
  {
    // B button released
    digitalWrite(LED_BUILTIN, LOW);
    bButtonDown = false;
    buttonStates &= B01111111;
  }

  // Touchpad
  touchpadX = analogRead(ANALOG_X_PIN);
  touchpadY = analogRead(ANALOG_Y_PIN);
  touchpadX -= ANALOG_X_CENTER;
  touchpadY -= ANALOG_Y_CENTER;

  // X axis
  if ((touchpadX < ANALOG_DEAD && touchpadX > 0) || (touchpadX > -ANALOG_DEAD && touchpadX < 0))
  {
    // Dead zone
    touchpadX = 0;
  }
  else if (touchpadX >= x_pos_range)
  {
    touchpadX = -1;
  }
  else if (touchpadX <= x_neg_range)
  {
    touchpadX = 1;
  }
  else
  {
    if (touchpadX >= 0)
    {
     // Positive range
     touchpadX /= (float) -x_pos_range;
    }
    else
    {
     // Negative range
     touchpadX /= (float) x_neg_range;
    }
  }

  //Y axis
  if ((touchpadY < ANALOG_DEAD && touchpadY > 0) || (touchpadY > -ANALOG_DEAD && touchpadY < 0))
  {
    // Dead zone
    touchpadY = 0;
  }
  else if (touchpadY >= y_pos_range)
  {
    touchpadY = 1;
  }
  else if (touchpadY <= y_neg_range)
  {
    touchpadY = -1;
  }
  else
  {
    if (touchpadY >= 0)
    {
     // Positive range
     touchpadY /= (float) y_pos_range;
    }
    else
    {
     // Negative range
     touchpadY /= (float) -y_neg_range;
    }
  }
}


