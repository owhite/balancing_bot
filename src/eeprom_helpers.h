#include <EEPROM.h>

// put PID in EEPROM so they are kept when arduino goes off
// this has been modified to work in teensy-land - where the sizeof(double)
// is different than an uno. 

void eeput(double value, int location) { 
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    byte b = *p++;
    EEPROM.write(location + i, b);
  }
}

double eeget(int location) { 
  double value;
  char * addr = (char * ) &value;
  for(int i=0; i<sizeof(value); i++) {
    uint8_t x = EEPROM.read(location + i);
    addr[i]=x;
  }
  return value;
}

void test() { 
  Serial.print("storing ");
  Serial.println(kp);
  eeput(kp,0);
  
  Serial.println("getting...");
  double x = eeget(0);

  Serial.print("got: ");
  Serial.println(x);
}

void writetoEEPROM() { 
  double x = 1.0;
  uint8_t inc = sizeof(x);
  eeput(kp,0);
  eeput(ki,inc);
  eeput(kd,inc * 2);
  double cks=0;
  for(int i=0; i<inc * 3; i++) cks+=EEPROM.read(i);
  eeput(cks,inc * 3);
}

void recoverPIDfromEEPROM() {
  double cks=0;
  double cksEE;
  uint8_t inc = sizeof(cks);
  kp=eeget(0);
  ki=eeget(inc);
  kd=eeget(inc * 2);

  for(int i=0; i<inc * 3; i++) cks+=EEPROM.read(i);
  cksEE=eeget(inc * 3);
  if(cks==cksEE) {
    kp=eeget(0);
    ki=eeget(inc);
    kd=eeget(inc * 2);
  }
  else Serial.println(F("*** Bad checksum"));

  pid.SetTunings(kp, ki, kd);
}


