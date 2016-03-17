
/*****************************************************************
  LSM9DS1_Basic_I2C.ino
  SFE_LSM9DS1 Library Simple Example Code - I2C Interface
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: April 30, 2015
  https://github.com/sparkfun/LSM9DS1_Breakout

  The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
  accelerometer, gyroscope, and magnetometer. Very cool! Plus it
  functions over either SPI or I2C.

  This Arduino sketch is a demo of the simple side of the
  SFE_LSM9DS1 library. It'll demo the following:
  How to create a LSM9DS1 object, using a constructor (global
  variables section).
  How to use the begin() function of the LSM9DS1 class.
  How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
  How to calculate actual acceleration, rotation speed,
  magnetic field strength using the calcAccel(), calcGyro()
  and calcMag() functions.
  How to use the data from the LSM9DS1 to calculate
  orientation and heading.

  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C or SPI. This example demonstrates how
  to use I2C. The pin-out is as follows:
	LSM9DS1 --------- Arduino
	 SCL ---------- SCL (A5 on older 'Duinos')
	 SDA ---------- SDA (A4 on older 'Duinos')
	 VDD ------------- 3.3V
	 GND ------------- GND
  (CSG, CSXM, SDOG, and SDOXM should all be pulled high.
  Jumpers on the breakout board will do this for you.)

  The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
  off the 3.3V rail! I2C pins are open-drain, so you'll be
  (mostly) safe connecting the LSM9DS1's SCL and SDA pins
  directly to the Arduino.

  Development environment specifics:
	IDE: Arduino 1.6.3
	Hardware Platform: SparkFun Redboard
	LSM9DS1 Breakout Version: 1.0

  This code is beerware. If you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!

  Distributed as-is; no warranty is given.
*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

// Libraries for MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
LSM9DS1 imu1;
LSM9DS1 imu2;
MPU6050 imu3(0x68);

int16_t ax, ay, az;
int16_t gx, gy, gz;

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1;
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2;
float ax3, ay3, az3, gx3, gy3, gz3;

float acc1x_avg = 0;
float acc1y_avg = 0;
float acc2x_avg = 0;
float acc2y_avg  = 0;

int state = 0;
int prev_state = 0;

uint32_t curr_time = 0;
uint32_t prev_time = 0;
uint32_t prev_control = 0;
uint32_t lastTurnOn = 0;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M_1	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG_1	0x6B // Would be 0x6A if SDO_AG is LOW

#define LSM9DS1_M_2  0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG_2  0x6A // Would be 0x6A if SDO_AG is LOW

#define LED_PIN 13
#define PUMP_L 22
#define PUMP_R 26
#define VALVE_L 24
#define VALVE_R 28

#define ACC_FSR (8192*2)
#define GYR_FSR (131*2)
#define sigFig 8

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define IMU1_ENABLE
#define IMU2_ENABLE
//#define IMU3_ENABLE
#define PRINT_SPEED 300 // 50 ms between prints
#define CONTRL_SPEED 500 // 50 ms between prints
#define PRINT_DATA
//#define CONTROL_DEBUGG
//#define CONTROL_DEBUG

void setup()
{
  // disable all actuators
  pinMode(PUMP_L, OUTPUT);
  pinMode(PUMP_R, OUTPUT);
  pinMode(VALVE_L, OUTPUT);
  pinMode(VALVE_R, OUTPUT);
  digitalWrite(PUMP_L, false);
  digitalWrite(VALVE_L, false);
  digitalWrite(PUMP_R, false);
  digitalWrite(VALVE_R, false);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  imu1.settings.device.commInterface = IMU_MODE_I2C;
  imu1.settings.device.mAddress = LSM9DS1_M_1;
  imu1.settings.device.agAddress = LSM9DS1_AG_1;
  imu2.settings.device.commInterface = IMU_MODE_I2C;
  imu2.settings.device.mAddress = LSM9DS1_M_2;
  imu2.settings.device.agAddress = LSM9DS1_AG_2;

  bool badInit = false;

#ifdef IMU1_ENABLE
  if (!imu1.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.IMU1.");
    badInit = true;
  } else {
    imu1.calibrate(false, 1);
    //Serial.print(imu1.aBiasRaw[0]);Serial.print(", ");Serial.print(imu1.aBiasRaw[1]);Serial.print(", ");Serial.println(imu1.aBiasRaw[2]);
    //Serial.print(imu1.gBiasRaw[0]);Serial.print(", ");Serial.print(imu1.gBiasRaw[1]);Serial.print(", ");Serial.println(imu1.gBiasRaw[2]);
    //Serial.println("Successfully communicated with LSM9DS1.INU1");
  }
#endif
#ifdef IMU2_ENABLE
  if (!imu2.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.IMU2");
    badInit = true;
  } else {
    imu2.calibrate(false, 2);
    //Serial.print(imu2.aBiasRaw[0]);Serial.print(", ");Serial.print(imu2.aBiasRaw[1]);Serial.print(", ");Serial.println(imu2.aBiasRaw[2]);
    //Serial.print(imu2.gBiasRaw[0]);Serial.print(", ");Serial.print(imu2.gBiasRaw[1]);Serial.print(", ");Serial.println(imu2.gBiasRaw[2]);
    //Serial.println("Successfully communicated with LSM9DS1.IMU2");
  }
#endif
#ifdef IMU3_ENABLE
  imu3.initialize();
  if (!imu3.testConnection())
  {
    Serial.println("Failed to communicate with MPU9050.IMU3");
    badInit = true;
  } else {
    //Serial.println("Successfully communicated with MPU9050.IMU3");
  }
#endif
  if (badInit) {
    while (1) ;
  }
  prev_time = millis();
}

void loop()
{
  curr_time = millis();
  if (curr_time - prev_time >= PRINT_SPEED) {

#ifdef IMU1_ENABLE
    imu1.getMotion9(&ax1, &ay1, &az1, &gx1, &gy1, &gz1, &mx1, &my1, &mz1);
#endif

#ifdef IMU2_ENABLE
    imu2.getMotion9(&ax2, &ay2, &az2, &gx2, &gy2, &gz2, &mx2, &my2, &mz2);
#endif

#ifdef IMU3_ENABLE
    imu3.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax3 = (float) ax / ACC_FSR; ay3 = (float) ay / ACC_FSR; az3 = (float) az / ACC_FSR;
    gx3 = (float) gx / GYR_FSR; gy3 = (float) gy / GYR_FSR; gz3 = (float) gz / GYR_FSR;
#endif

#ifdef PRINT_DATA
      //Serial.print(curr_time); Serial.print(", ");

/*      
      #ifdef IMU2_ENABLE
      Serial.print(ax2, sigFig); Serial.print(", "); Serial.print(ay2, sigFig); Serial.print(", "); Serial.print(az2, sigFig); Serial.print(", ");
      Serial.print(gx2, sigFig); Serial.print(", "); Serial.print(gy2, sigFig); Serial.print(", "); Serial.print(gz2, sigFig); Serial.print(", ");
      //Serial.print(mx2, sigFig); Serial.print(", "); Serial.print(my2, sigFig); Serial.print(", "); Serial.print(mz2, sigFig); Serial.print(", ");
      #endif
      #ifdef IMU1_ENABLE
      Serial.print(ax1, sigFig); Serial.print(", "); Serial.print(ay1, sigFig); Serial.print(", "); Serial.print(az1, sigFig); Serial.print(", ");
      Serial.print(gx1, sigFig); Serial.print(", "); Serial.print(gy1, sigFig); Serial.print(", "); Serial.print(gz1, sigFig); Serial.print(", ");
      //Serial.print(mx1, sigFig); Serial.print(", "); Serial.print(my1, sigFig); Serial.print(", "); Serial.print(mz1, sigFig); Serial.print(", ");
      #endif
*/    
    
#ifdef IMU1_ENABLE
    Serial.print(-gy1, sigFig); Serial.print(", ");
    Serial.print(-gz1, sigFig); Serial.print(", ");
    Serial.print(gx1, sigFig); Serial.print(", ");
    Serial.print(-ay1, sigFig); Serial.print(", ");
    Serial.print(-az1, sigFig); Serial.print(", ");
    Serial.print(ax1, sigFig); Serial.print(", ");
    //Serial.print(mx1, sigFig); Serial.print(", "); Serial.print(my1, sigFig); Serial.print(", "); Serial.print(mz1, sigFig); Serial.print(", ");
#endif
#ifdef IMU2_ENABLE
    Serial.print(-gz2, sigFig); Serial.print(", ");
    Serial.print(gx2, sigFig); Serial.print(", ");
    Serial.print(-gy2, sigFig); Serial.print(", ");
    Serial.print(-az2, sigFig); Serial.print(", ");
    Serial.print(ax2, sigFig); Serial.print(", ");
    Serial.print(-ay2, sigFig); Serial.print(", ");
    //Serial.print(mx2, sigFig); Serial.print(", "); Serial.print(my2, sigFig); Serial.print(", "); Serial.print(mz2, sigFig); Serial.print(", ");
#endif


#ifdef IMU3_ENABLE
    Serial.print(ax3, sigFig); Serial.print(", "); Serial.print(ay3, sigFig); Serial.print(", "); Serial.print(az3, sigFig); Serial.print(", ");
    Serial.print(gx3, sigFig); Serial.print(", "); Serial.print(gy3, sigFig); Serial.print(", "); Serial.print(gz3, sigFig);
#endif
    Serial.println("");
#endif

    //-----CONTROLLER-----
    /* Demo using IMU3 */
    /*
      if (az3 >= 0) {
      actuator_left = true;
      //Serial.println("Turning on Left Actuator");
      } else {
      actuator_left = false;
      //Serial.println("Turning off Left Actuator");
      }
      if (az2 >= 0) {
      actuator_right = true;
      //Serial.println("Turning on Right Actuator");
      } else {
      actuator_right = false;
      //Serial.println("Turning off Right Actuator");
      }
    */
    /* Real Controller */
    if (acc2x_avg == 0 && acc2y_avg == 0) {
      acc2x_avg = ax1;
      acc2y_avg = ay1;
      acc1x_avg = ax2;
      acc1y_avg = ay2;
    } else {
      /*
      acc2x_avg = (2*acc2x_avg + ax1) / 3;
      acc2y_avg = (2*acc2y_avg + ay1) / 3;
      acc1x_avg = (2*acc1x_avg + ax2) / 3;
      acc1y_avg = (2*acc1y_avg + ay2) / 3;
      */
      acc2x_avg = ax1;
      acc2y_avg = ay1;
      acc1x_avg = ax2;
      acc1y_avg = ay2;
    }
    //Nan Controller 
      if (acc2x_avg < 2.30695) {
        if (acc2x_avg < 0.0653505) {
          state = 1;
        } else {
          if (acc2x_avg < 2.16984) {
            state = 0;
          } else {
            if (acc2y_avg < 8.94368) {
              state = 0;
            } else {
              state = -1;
            }
          }
        }
      } else {
        state = -1;
      }
      
    /* David Controller 
    if (acc1x_avg < -0.743337) {
      state = 1;
    } else {
      if (acc2x_avg < 1.84257) {
        if (acc1x_avg < -0.130464) {
          if (acc1y_avg < 8.22781) {
            state = 0;
          } else {
            state = 1;
          }
        } else {
          state = 0;
        }
      } else {
        if (acc1y_avg < 10.376) {
          state = -1;
        } else {
          state = 0;
        }
      }
    }
    */
    //Ensure the actuator do not turn on for more than 1s
    //-----ACTUATION-----
    if (millis() - prev_control >= CONTRL_SPEED) {
      prev_control = millis();
      if (state != prev_state) {
        if (state == 0) {
          digitalWrite(VALVE_L, false);
          digitalWrite(PUMP_L, false);
          digitalWrite(VALVE_R, false);
          digitalWrite(PUMP_R, false);
#ifdef CONTROL_DEBUGG
          Serial.println("Turn OFF");
#endif
        } else if (state == 1) {
          digitalWrite(VALVE_L, true);
          digitalWrite(PUMP_L, true);
#ifdef CONTROL_DEBUGG
          Serial.println("Turn on Left");
#endif
          lastTurnOn = millis();
        } else if (state == -1) {
          digitalWrite(VALVE_R, true);
          digitalWrite(PUMP_R, true);
#ifdef CONTROL_DEBUGG
          Serial.println("Turn on Right");
#endif
          lastTurnOn = millis();
        }
        prev_state = state;
      } else {
        if (state != 0 && (millis() - lastTurnOn > 999)) {
          digitalWrite(VALVE_L, false);
          digitalWrite(PUMP_L, false);
          digitalWrite(VALVE_R, false);
          digitalWrite(PUMP_R, false);
          prev_state = 0;
#ifdef CONTROL_DEBUGG
          Serial.println("Longer than 1 sec: Turn OFF");
#endif
        }
      }
    }
#ifdef CONTROL_DEBUG
    Serial.print(avgGravMag1, sigFig); Serial.print(", "); Serial.print(avgGravMag2, sigFig); Serial.print(", ");
    Serial.print(state); Serial.println(", ");
#endif
    /*
        if (state && (millis() - lastTurnOn > 1000)){
             state = 0;
             Serial.println("Longer than 1 sec, Turning OFF");
        }
    */
    prev_time = curr_time;
  }
}



