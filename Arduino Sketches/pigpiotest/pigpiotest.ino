#include <Wire.h>

typedef union
{
  float numbers[4];
  byte bytes[16];
} FLOATUNION;

typedef union
{
  short number;
  byte bytes[2]; 
} SHORTUNION;

FLOATUNION state;
FLOATUNION K;
SHORTUNION ctrl;

float x = 1.0f;

void setup()
{
   Wire.begin(); // join i2c bus as master
   Serial.begin(57600);
}

void loop()
{
  
  for (int i=0; i<4; i++)
  {
    state.numbers[i] = x;
  }
  ctrl.number = -220;
  

  Wire.beginTransmission(69); // transmit to device 69
  Wire.write(state.bytes, 16);
  Wire.endTransmission();
  delay(100);
  Wire.beginTransmission(69);
  Wire.write(ctrl.bytes, 2);
  Wire.endTransmission();

  Wire.requestFrom(69, 16);
  int ctr = 0;
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read();
    K.bytes[ctr]=c;
    if(ctr>=15) break;
    ctr++;
  }
  
  x = x + 0.125;
  delay(500);
}
