// MPU9250 Basic Example Code by: Kris Winer
// https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino

#include "definitions.h"
#include "variables.h"
#include "eeprom_helpers.h"
#include <i2c_t3.h>
#include "I2c_funcs.h"
#include "quaternionFilter.h"
#include "MPU9250_helpers.h"
#include "PID_v1.h"

//Define Variables we'll be connecting to

//Specify the links and initial tuning parameters
PID myPID(&position, &output, &target, kd, ki, kp, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  position = analogRead(0);
  target = 100;

  Serial.begin(38400);
  pinMode(blinkPin, OUTPUT);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  blinkInterval = 1000;

}

void loop()
{
  position = analogRead(0);
  while(!myPID.Compute())
  analogWrite(3,output);

  ledUpdate();


}

void ledUpdate() {
  blinkNow = millis();
  if ((blinkNow - blinkDelta) > blinkInterval) {
    blinkOn = (blinkOn == 1) ? 0 : 1;
    digitalWrite(blinkPin, blinkOn);
    blinkDelta = blinkNow;
  }
}

