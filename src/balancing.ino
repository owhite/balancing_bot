// Kris Winer's MPU-6050 code
// https://github.com/kriswiner/MPU-6050
#include "definitions.h"
#include "variables.h"
#include <i2c_t3.h>
#include "I2c_funcs.h"
#include "quaternionFilter.h"
#include "MPU9250_helpers.h"
#include "PID_v1.h"

uint32_t tmp1;
uint32_t tmp2;
uint32_t tmp3;

PID pid(&position, &output, &target, kp, ki, kd, DIRECT);

#include "eeprom_helpers.h"


void setup() {
  Serial.begin(38400);
  Serial1.begin(38400);  

  target = defaultTarget;

  inputString.reserve(200);

  pinMode(blinkPin, OUTPUT);

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-255,255);
  recoverPIDfromEEPROM();
  pid.SetTunings(kp, ki, kd);

  resetMPU();
}

void loop() {  

  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    if (inChar == '\n') {
      handle_cmd();
    }
  }

  if (zeroToggle) { resetMPU(); }

  tmp2 = micros();
  MPUupdate();
  tmp1 = micros();

  double diff = abs(pitch - target);

  pid.Compute();

  tmp3 = tmp1 - tmp2;
  Serial.print(position);
  Serial.print(" ");
  Serial.print(output);
  Serial.print(" ");
  Serial.println(tmp3);

  direction = (output < 0.0) ? 1 : 2;

  power = 140 * pauseToggle;

  // ledUpdate();

  //  setMotorPosition((uint8_t) abs(output), direction, power,
  //		   (uint8_t) abs(output), direction, power, blinkOn);
}

void resetMPU() {
  Serial1.println("resetting...");

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1200);
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  
  if (c == 0x71)  {  
      MPU9250SelfTest(SelfTest); 
      calibrateMPU9250(gyroBias, accelBias); 
      delay(1000); 
      initMPU9250(); 
      byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  
      initAK8963(magCalibration);
  }
  else {
      Serial1.print("Could not connect to MPU9250: 0x");
      Serial1.println(c, HEX);
      while(1) ; // Loop forever if communication doesn't happen
  }
  Serial1.println("  done");

  zeroToggle = 0;
}

void MPUupdate () {
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { // enter on interrupt
    calcYawPitchRoll(); // 380 us

    Now = micros();
    // elapsed time since last filter update
    deltat = ((Now - lastUpdate)/1000000.0f); 
    lastUpdate = Now;
    // needed for filter update
    sum += deltat; 
    sumCount++;
  
    MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    pitch = (pitch * 180.0f / PI) + 180;
    position = pitch;
  }
}

// 900us
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

void ledUpdate() {
  blinkNow = millis();
  if ((blinkNow - blinkDelta) > blinkInterval) {
    blinkOn = (blinkOn == 1) ? 0 : 1;
    digitalWrite(blinkPin, blinkOn);
    blinkDelta = blinkNow;
    if (reportToggle == 1) {
      Serial1.print(target);
      Serial1.print(" ");
      Serial1.print(position);
      Serial1.print(" ");
      Serial1.println(output);
    }
  }
}

void handle_cmd() {
  inputString.trim(); // removes beginning and ending white spaces
  int idx = inputString.indexOf(' ');   
  String cmd = inputString.substring(0, idx); 
  String value = inputString.substring(idx + 1);

  if ((cmd.length() > 0)) {
    if (cmd.equals("probe_device")) {
      Serial1.println("balancing");
    }
    if (cmd.equals("p")) {
      Serial1.println("toggle pause");
      pauseToggle = (pauseToggle == 1) ? 0 : 1;
    }
    if (cmd.equals("zero")) {
      Serial1.println("zero system");
      zeroToggle = 1;
    }
    if (cmd.equals("report")) {
      Serial1.println("report toggle");
      reportToggle = (reportToggle == 1) ? 0 : 1;
    }
    if (cmd.equals("F")) {
      Serial1.println("forward");
    }
    if (cmd.equals("write")) {
      Serial1.println("write to EEPROM");
      writetoEEPROM();
    }
    if (cmd.equals("reset")) {
      Serial1.println("EEPROM get PIDs");
      recoverPIDfromEEPROM();
    }
    if (cmd.equals("get_PIDs")) {
      Serial1.print(kp);
      Serial1.print(" ");
      Serial1.print(ki);
      Serial1.print(" ");
      Serial1.println(kd);
    }
    if (cmd.equals("B")) {
      Serial1.println("set blink rate"); // good way to tell it's alive
      blinkInterval = value.toInt();
    }
    if (cmd.equals("upscale")) {
      Serial1.print("upscale ");
      float x = value.toFloat();
      Serial1.println(x);
      target = defaultTarget + x;
    }
    if (cmd.equals("downscale")) {
      Serial1.print("downscale ");
      float x = value.toFloat();
      Serial1.println(x);
      target = defaultTarget - x;
    }
    if (cmd.equals("P")) {
      Serial1.print("set P ");
      kp = value.toFloat();
      pid.SetTunings(kp, ki, kd);
      Serial1.println(kp);
    }
    if (cmd.equals("I")) {
      Serial1.print("set I ");
      ki = value.toFloat();
      pid.SetTunings(kp, ki, kd);
      Serial1.println(ki);
    }
    if (cmd.equals("D")) {
      Serial1.print("set D ");
      kd = value.toFloat();
      pid.SetTunings(kp, ki, kd);
      Serial1.println(kd);
    }
    inputString = "";
  }
}

void serialEvent() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    if (inChar == '\n') {
      handle_cmd();
    }
  }
}
