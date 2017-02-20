/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you
 find it useful you can buy me a beer some time.
 Modified by Brent Wilkins July 19, 2016

 Demonstrate basic MPU-9250 functionality including parameterizing the register
 addresses, initializing the sensor, getting properly scaled accelerometer,
 gyroscope, and magnetometer data out. Added display functions to allow display
 to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
 Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
 and the Teensy 3.1.

 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.

 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"

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


#define AHRS true         // Set to false for basic data read
#define SerialDebug false // Set to true to get Serial output for debugging
#define ReadableSerial false
// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

MPU9250 myIMU;

void setup()
{
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
  
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x73) // WHO_AM_I should always be 0x71
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM ");
    Serial.print(d, HEX);
    Serial.print(" I should be ");
    Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    // Set the previously measured calibration data
    myIMU.magBias[0] = 54.95f;
    myIMU.magBias[1] = 401.46f;
    myIMU.magBias[2] = -468.24f;
    myIMU.magScale[0] = 0.96f;
    myIMU.magScale[1] = 1.09f;
    myIMU.magScale[2] = 0.96f;
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
    //delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }
    
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);
  //customFilterUpdate(myIMU.gx, myIMU.gy, myIMU.gz, myIMU.ax, myIMU.ay, myIMU.az, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");
      }

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 4.47;
      myIMU.roll *= RAD_TO_DEG;

      if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
  ReadControllerData();
  if (!SerialDebug)
  {
    //SendSensorDataOnlyQuat();

    if (ReadableSerial)
      SendReadableDataOnSerial();
    else
      SendDataOnSerial();
  }
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
}

void SendSensorDataOnly()
{
    myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                  * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
    myIMU.pitch *= RAD_TO_DEG;
    myIMU.yaw   *= RAD_TO_DEG;

    myIMU.yaw  -= 8.5;
    myIMU.roll *= RAD_TO_DEG;
      
  Serial.print("&,");
  Serial.print(myIMU.yaw, 16);
  Serial.print(",");
  Serial.print(myIMU.pitch, 16);
  Serial.print(",");
  Serial.print(myIMU.roll,  16);
  Serial.print("\n");
}

void SendSensorDataOnlyQuat() {
  myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;

  myIMU.yaw  -= 8.5;
  myIMU.roll *= RAD_TO_DEG;
    
  Serial.print("&,");
  Serial.print(*getQ(),16);
  Serial.print(",");
  Serial.print(*(getQ() + 1),16);
  Serial.print(",");
  Serial.print(*(getQ() + 2),16);
  Serial.print(",");
  Serial.print(*(getQ() + 3),16);

  Serial.print(",");
  Serial.print(1000 * myIMU.ax);
  Serial.print(",");
  Serial.print(1000 * myIMU.ay);
  Serial.print(",");
  Serial.print(1000 * myIMU.az);
  Serial.print(",");
  Serial.print(myIMU.gx);
  Serial.print(",");
  Serial.print(myIMU.gy);
  Serial.print(",");
  Serial.print(myIMU.gz);
  Serial.print(",");
  Serial.print(myIMU.mx);
  Serial.print(",");
  Serial.print(myIMU.my);
  Serial.print(",");
  Serial.print(myIMU.mz);

  Serial.print(",");
  Serial.print(myIMU.yaw,8);
  Serial.print(",");
  Serial.print(myIMU.pitch,8);
  Serial.print(",");
  Serial.print(myIMU.roll,8);
  
  
  Serial.println('|');
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
  Serial.print(*getQ(),4);
  Serial.print(";");
  Serial.print(*(getQ() + 1),4);
  Serial.print(";");
  Serial.print(*(getQ() + 2),4);
  Serial.print(";");
  Serial.print(*(getQ() + 3),4);
  
  Serial.println('|');
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

  Serial.print("; q0 = ");  Serial.print(*getQ());
  Serial.print(" qx = "); Serial.print(*(getQ() + 1));
  Serial.print(" qy = "); Serial.print(*(getQ() + 2));
  Serial.print(" qz = "); Serial.print(*(getQ() + 3));
  
  Serial.print('\n');
}

