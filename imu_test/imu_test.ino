/* FILE:    ARD_MPU9250_Example
   DATE:    22/05/14
   VERSION: 0.1
   
REVISIONS:

22/05/14 Created version 0.1

This is an example of how to use the Hobby Components MPU9250 accelerometer, gyro, 
compass (magnetometer) module (HCMODU0092). The MPU9150 has many advanced features
however this example sketch is written to show the basic steps required to obtain reading from 
all three sensors and the additional temperature sensor. The sketch will repeatedly read 
measurements from each axis of all three sensors plus the additional temperature sensor and 
output them to the serial port.

PINOUT:

MODULE`                Arduino
VCC                    +5V
GND                    GND
SCL                    A5*
SDA                    A4*
EDA                    N/A
ECL                    N/A
AD0                    N/A
INT                    N/A

*Please note that the MPU9250 operates at 3.3V (via a 3.3V regulator) and these 
pins should not be driven above 3.8V therefore you may require level shifters to
ensure safe operation. These two pins also include 220R pull-up resistors.

You may copy, alter and reuse this code in any way you like, but please leave
reference to HobbyComponents.com in your comments if you redistribute this code.
This software may not be used for the purpose of promoting or selling products 
that directly compete with Hobby Components Ltd's own range of products.

THIS SOFTWARE IS PROVIDED "AS IS". HOBBY COMPONENTS MAKES NO WARRANTIES, WHETHER
EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ACCURACY OR LACK OF NEGLIGENCE.
HOBBY COMPONENTS SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR ANY DAMAGES,
INCLUDING, BUT NOT LIMITED TO, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY
REASON WHATSOEVER.
*/

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

/* Include the standard wire library */
#include <Wire.h>

void setup()
{
  /* Initialise the I2C bus */
  Wire.begin();  
  
  /* Initialise the serial interface */
  Serial.begin(9600);
  
  /* Initialise the accelerometer and gyro and put the I2C bus into pass-through mode*/
  Initalise_AccelGyro(ACCELRANGE_8g, GYRORANGE_2000DPS);
}

/* Main program */
void loop()
{
  
  /* Read the temperature sensor and send it to the serial port */
  Serial.print("Temp: ");  
  Serial.print((double)(Read_Acc_Gyro(TEMP_OUT_H) + 11900) / 340);
  Serial.print(" ");
  
  /* Read the accelerometer X, Y, and Z axis and send it to the serial port */
  Serial.print("Acc X: "); 
  Serial.print(Read_Acc_Gyro(ACCEL_XOUT_H));
  Serial.print(" Acc Y: ");
  Serial.print(Read_Acc_Gyro(ACCEL_YOUT_H));
  Serial.print(" Acc Z: ");
  Serial.print(Read_Acc_Gyro(ACCEL_ZOUT_H));
  
  /* Read the gyroscope X, Y, and Z axis and send it to the serial port */
  Serial.print(" Gyro X: ");
  Serial.print(Read_Acc_Gyro(GYRO_XOUT_H));
  Serial.print(" Gyro Y: ");
  Serial.print(Read_Acc_Gyro(GYRO_YOUT_H));
  Serial.print(" Gyro Z: ");
  Serial.print(Read_Acc_Gyro(GYRO_ZOUT_H));

  /* Trigger a compass measurement */
  Trigger_Compass();
  
  /* Read the compass X, Y, and Z axis and send it to the serial port */
  Serial.print(" Comp X: ");
  Serial.print(Read_Compass(COMP_XOUT_L));
  Serial.print(" Comp Y: ");
  Serial.print(Read_Compass(COMP_YOUT_L));
  Serial.print(" Comp Z: ");
  Serial.println(Read_Compass(COMP_ZOUT_L));
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
