#include <Wire.h>

/* I2C addresses of Accelerometer/Gyro and Compass */
#define I2CACCGYROADD 0x68 
#define I2CCOMPADD 0x0C

/* Accelerometer/Gyro register addresses */
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47
#define PWR_MGMT_1 0x6B

/*Compass register addresses */
#define COMP_STATUS 0x02
#define COMP_XOUT_L 0x03
#define COMP_YOUT_L 0x05
#define COMP_ZOUT_L 0x07

/* Accelerometer range modes */
#define ACCELRANGE_2g 0
#define ACCELRANGE_4g 1
#define ACCELRANGE_8g 2
#define ACCELRANGE_16g 3

/* Gyroscope sensitivity */
#define GYRORANGE_250DPS 0
#define GYRORANGE_500DPS 1
#define GYRORANGE_1000DPS 2
#define GYRORANGE_2000DPS 3

// Button pinout
#define MENU_BUTTON_PIN 9
#define SYSTEM_BUTTON_PIN 7
#define GRIP_BUTTON_PIN 4
#define TRIGGER_BUTTON_PIN 2
#define TOUCHPAD_PRESS_PIN 3 
#define ANALOG_X_PIN A6
#define ANALOG_Y_PIN A7
#define POWER_BUTTON_PIN 10

// Analog correction values
#define ANALOG_DEAD 10
#define ANALOG_X_CENTER 494
#define ANALOG_X_MIN 0
#define ANALOG_X_MAX 1023
#define ANALOG_Y_CENTER 437
#define ANALOG_Y_MIN 0
#define ANALOG_Y_MAX 500

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
int buttonStates = 0;
int lastButtonStates = 0;
float touchpadX = 0;
float touchpadY = 0;
bool powerButtonDown = false;
char handLetter = 'R';

// the setup function runs once when you press reset or power the board
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
  
  // Calculate initial analog values
  x_pos_range = ANALOG_X_MAX - ANALOG_X_CENTER;
  x_neg_range = ANALOG_X_MIN - ANALOG_X_CENTER;
  y_pos_range = ANALOG_Y_MAX - ANALOG_Y_CENTER;
  y_neg_range = ANALOG_Y_MIN - ANALOG_Y_CENTER;

  /* Initialise the I2C bus */
  Wire.begin(); 

    /* Initialise the accelerometer and gyro and put the I2C bus into pass-through mode*/
  Initalise_AccelGyro(ACCELRANGE_8g, GYRORANGE_2000DPS);
  
  Serial.begin(115200, SERIAL_8N1); 
  Serial.print("#Program started");
}

// the loop function runs over and over again forever
void loop() {
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
  else if (touchpadX >= (ANALOG_X_MAX - ANALOG_X_CENTER))
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
  else if (touchpadY >= (ANALOG_Y_MAX - ANALOG_Y_CENTER))
  {
    touchpadY = 1;
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

  /* Trigger a compass measurement */
  Trigger_Compass();
  SendDataOnSerial();
  delay(5);
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
  
  /* Read the accelerometer X, Y, and Z axis and send it to the serial port */
  Serial.print(";"); 
  Serial.print(Read_Acc_Gyro(ACCEL_XOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(ACCEL_YOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(ACCEL_ZOUT_H));

    /* Read the gyroscope X, Y, and Z axis and send it to the serial port */
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(GYRO_XOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(GYRO_YOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(GYRO_ZOUT_H));
  
  /* Read the compass X, Y, and Z axis and send it to the serial port */
  Serial.print(";");
  Serial.print(Read_Compass(COMP_XOUT_L));
  Serial.print(";");
  Serial.print(Read_Compass(COMP_YOUT_L));
  Serial.print(";");
  Serial.print(Read_Compass(COMP_ZOUT_L));
  
  Serial.print('|');
}

void SendReadableDataOnSerial()
{
  Serial.print(handLetter);
  Serial.print(";");
  Serial.print(buttonStates);
  Serial.print("; X:");
  Serial.print(touchpadX);
  Serial.print("; Y: ");
  Serial.print(touchpadY);
  
  /* Read the accelerometer X, Y, and Z axis and send it to the serial port */
  Serial.print("; Acc XYZ: "); 
  Serial.print(Read_Acc_Gyro(ACCEL_XOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(ACCEL_YOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(ACCEL_ZOUT_H));

    /* Read the gyroscope X, Y, and Z axis and send it to the serial port */
  Serial.print("; Gyro XYZ: ");
  Serial.print(Read_Acc_Gyro(GYRO_XOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(GYRO_YOUT_H));
  Serial.print(";");
  Serial.print(Read_Acc_Gyro(GYRO_ZOUT_H));
  
  /* Read the compass X, Y, and Z axis and send it to the serial port */
  Serial.print("; Comp XYZ: ");
  Serial.print(Read_Compass(COMP_XOUT_L));
  Serial.print(";");
  Serial.print(Read_Compass(COMP_YOUT_L));
  Serial.print(";");
  Serial.print(Read_Compass(COMP_ZOUT_L));
  
  Serial.print('\n');
}

/* Read one of the accelerometer or gyro axis registers */
int Read_Acc_Gyro(byte axis)
{
  int Data;
   
  /* Select the required register */ 
  Wire.beginTransmission(I2CACCGYROADD); 
  Wire.write(axis); 
  Wire.endTransmission(); 
  
  /* Request the high and low bytes for the required axis */
  Wire.requestFrom(I2CACCGYROADD, 2);
  Data = (int)Wire.read() << 8;
  Data = Data | Wire.read();
  Wire.endTransmission(); 
  
  return Data;
}


/* Initialises the accelerometer and gyro to one of the sensitivity 
   ranges and puts the I2C bus into pass-through mode */
void Initalise_AccelGyro(byte Accel_Range, byte Gyro_Range)
{
  /* Take the MPU9150 out of sleep */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(PWR_MGMT_1); 
  Wire.write(0); 
  Wire.endTransmission(); 
  
  /* Set the sensitivity of the module */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(ACCEL_CONFIG); 
  Wire.write(Accel_Range << 3); 
  Wire.endTransmission(); 
  
  /* Set the sensitivity of the module */
  Wire.beginTransmission(I2CACCGYROADD);
  Wire.write(GYRO_CONFIG); 
  Wire.write(Gyro_Range << 3); 
  Wire.endTransmission(); 
  
  /* Put the I2C bus into pass-through mode so that the aux I2C interface
     that has the compass connected to it can be accessed */
  Wire.beginTransmission(I2CACCGYROADD); 
  Wire.write(0x6A); 
  Wire.write(0x00); 
  Wire.endTransmission(true);

  Wire.beginTransmission(I2CACCGYROADD); 
  Wire.write(0x37); 
  Wire.write(0x02); 
  Wire.endTransmission(true); 
}

/* Read one of the compass axis */
int Read_Compass(byte axis)
{
  int Data;
 
  /* Select the required axis register */
  Wire.beginTransmission(I2CCOMPADD); 
  Wire.write(axis); 
  Wire.endTransmission(); 
 
  /* Request the low and high bytes for the required axis */
  Wire.requestFrom(I2CCOMPADD, 2);
  Data = Wire.read();
  Data = Data | (int)(Wire.read() << 8);
  Wire.endTransmission(); 
  
  return Data;
}


/* Trigger a single shot compass reading of all three axis */
void Trigger_Compass(void)
{
  
  /* Trigger a measurement */
  Wire.beginTransmission(I2CCOMPADD); 
  Wire.write(0x0A); 
  Wire.write(0x01); 
  Wire.endTransmission(true);
  
  /* Wait for the measurement to complete */
  do
  {
    Wire.beginTransmission(I2CCOMPADD); 
    Wire.write(COMP_STATUS); 
    Wire.endTransmission(); 
 
    Wire.requestFrom(I2CCOMPADD, 1);
  }while(!Wire.read()); 
}
