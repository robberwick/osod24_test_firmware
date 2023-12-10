#include <cstdio>
#include "pico/stdlib.h"
#include "navigator.h"
#include "receiver.h"
#include "statemanager.h"
#include "motor2040.hpp"
#include "tank_steer_strategy.h"


#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080

BNO080 myIMU;

int main() {
  stdio_init_all();

  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1)
      ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms

  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw"));
  while (1){
    //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
        float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
        float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
        float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

        Serial.print(roll, 1);
        Serial.print(F(","));
        Serial.print(pitch, 1);
        Serial.print(F(","));
        Serial.print(yaw, 1);

        Serial.println();
    }
  }
}
