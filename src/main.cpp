
/*
  Using the BNO08x IMU

  Example : Euler Angles
  By: Paul Clark
  Date: April 28th, 2020

  This example shows how to output the Euler angles: roll, pitch and yaw.
  The yaw (compass heading) is tilt-compensated, which is nice.
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440

  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Originally written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  Adjusted by Pete Lewis @ SparkFun Electronics, June 2023 to incorporate the
  CEVA Sensor Hub Driver, found here:
  https://github.com/ceva-dsp/sh2

  Also, utilizing code from the Adafruit BNO08x Arduino Library by Bryan Siepert
  for Adafruit Industries. Found here:
  https://github.com/adafruit/Adafruit_BNO08x

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN

  Serial.print it out at 115200 baud to serial monitor.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/22857
*/

//#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed


#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "bno080.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21

BNO08x myIMU;

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  printf("Setting desired reports\n");
  if (myIMU.enableRotationVector() == true) {
    printf("Rotation vector enabled\n");
    printf("Output in form roll, pitch, yaw\n");
  } else {
    printf("Could not enable rotation vector\n");
  }
}

int main() {
  stdio_init_all();

  i2c_inst_t* my_i2c_port = i2c_default; // or i2c0, i2c1, etc.

  i2c_init(my_i2c_port, 100 * 1000);

  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);


  printf("BNO08x Read Example\n");



  //if (myIMU.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (myIMU.begin(BNO08X_ADDR, my_i2c_port)) {
    printf("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...\n");
    while (1)
      ;
  }
  printf("BNO08x found!\n");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  printf("Reading events\n");
  sleep_ms(100);




  while(1) {
    sleep_ms(10);

    if (myIMU.wasReset()) {
      printf("sensor was reset ");
      setReports();
    }

    // Has a new event come in on the Sensor Hub Bus?
    if (myIMU.getSensorEvent() == true) {

      // is it the correct sensor data we want?
      if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {

        float roll = (myIMU.getRoll()) * 180.0 / M_PI; // Convert roll to degrees
        float pitch = (myIMU.getPitch()) * 180.0 / M_PI; // Convert pitch to degrees
        float yaw = (myIMU.getYaw()) * 180.0 / M_PI; // Convert yaw / heading to degrees

        printf("Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n", roll, pitch, yaw);

      }
    }
  }
}