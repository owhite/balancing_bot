// Kris Winer's MPU-6050 code
// https://github.com/kriswiner/MPU-6050
#include "definitions.h"
#include "variables.h"
#include "eeprom_helpers.h"
#include <i2c_t3.h>
#include "I2c_funcs.h"
#include "quaternionFilter.h"
#include "MPU_helpers.h"
#include "PID_v1.h"

PID pid(&position, &output, &target, kp, ki, kd, DIRECT);

void setup() {
  inputString.reserve(200);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 1600000);
  Wire.setDefaultTimeout(200000); // 200ms

  Serial.begin(38400);
  
  pinMode(blinkPin, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-255,255);
  recoverPIDfromEEPROM();
  pid.SetTunings(kp, ki, kd);

  testAndCalibrateIMU();
  blinkDelta = millis();

  setMotorPosition(0, FORWARD, 140, 0, FORWARD, 140, 0);

  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) { calcYawPitchRoll(); }  

  while(pitch > 185 || pitch < 175) {
    if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
      calcYawPitchRoll();
    }  
    ledUpdate();
  }
  blinkInterval = blinkNormal; 
  Serial.println(" balanced ");

  digitalWrite(blinkPin, HIGH);
}

void loop() {  
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
    calcYawPitchRoll();
    position = pitch;
  }  

  ledUpdate();
  while(!pid.Compute()); // wait to compute PID
  direction = (output < 0.0) ? FORWARD : REVERSE;
  power = (pauseToggle != 1) ? 140 : 0;

  setMotorPosition((uint8_t) abs(output), direction, power,
		   (uint8_t) abs(output), direction, power, blinkOn);
}

void ledUpdate() {
  blinkNow = millis();
  if ((blinkNow - blinkDelta) > blinkInterval) {
    blinkOn = (blinkOn == 1) ? 0 : 1;
    digitalWrite(blinkPin, blinkOn);
    blinkDelta = blinkNow;
    if (reportToggle == 1) {
      Serial.print(position);
      Serial.print(" ");
      Serial.println(output);
    }
  }
}

void setMotorPosition(uint8_t speed1, uint8_t dir1, uint8_t power1,
		      uint8_t speed2, uint8_t dir2, uint8_t power2,
		      uint8_t LED_state) {
  Wire.beginTransmission(SLAVE_ADDRESS);

  // this sends data to motor
  Wire.write(speed1);
  Wire.write(dir1);
  Wire.write(power1);

  Wire.write(speed2);
  Wire.write(dir2);
  Wire.write(power2);

  // also sends data to LED
  Wire.write(LED_state);

  Wire.endTransmission();

  // useful for debugging wire parameters
  if(Wire.getError()) {
    wireFailures++;
  }
  wireAttempts++;
  delayMicroseconds(2); 
}

void handle_cmd() {
  inputString.trim(); // removes beginning and ending white spaces
  int idx = inputString.indexOf(' ');   
  String cmd = inputString.substring(0, idx); 
  String value = inputString.substring(idx + 1);

  if ((cmd.length() > 0)) {
    if (cmd.equals("probe_device")) {
      Serial.println("balancing");
    }
    if (cmd.equals("p")) {
      Serial.println("toggle pause");
      pauseToggle = (pauseToggle == 1) ? 0 : 1;
    }
    if (cmd.equals("zero")) {
      Serial.println("zero system");
    }
    if (cmd.equals("report")) {
      Serial.println("report toggle");
      reportToggle = (reportToggle == 1) ? 0 : 1;
    }
    if (cmd.equals("F")) {
      Serial.println("forward");
    }
    if (cmd.equals("write")) {
      Serial.println("write to EEPROM");
      writetoEEPROM();
    }
    if (cmd.equals("reset")) {
      Serial.println("EEPROM get PIDs");
      recoverPIDfromEEPROM();
    }
    if (cmd.equals("get_PIDs")) {
      Serial.print(kp);
      Serial.print(" ");
      Serial.print(ki);
      Serial.print(" ");
      Serial.println(kd);
    }
    if (cmd.equals("B")) {
      Serial.println("set blink rate"); // good way to tell it's alive
      blinkInterval = value.toInt();
    }
    if (cmd.equals("P")) {
      Serial.print("set P ");
      kp = value.toFloat();
      Serial.println(kp);
    }
    if (cmd.equals("I")) {
      Serial.print("set I ");
      ki = value.toFloat();
      Serial.println(ki);
    }
    if (cmd.equals("D")) {
      Serial.print("set D ");
      kd = value.toFloat();
      Serial.println(kd);
    }
    inputString = "";
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      handle_cmd();
    }
  }
}
