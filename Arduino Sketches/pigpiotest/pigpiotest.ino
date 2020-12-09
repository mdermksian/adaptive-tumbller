#include <Wire.h>

void setup()
{
   Wire.begin(); // join i2c bus as master
   Serial.begin(57600);
}

typedef union
{
  float numbers[4];
  byte bytes[16];
} FLOATUNION;

FLOATUNION outgoing;
FLOATUNION incoming;

float x = 1.0f;

void loop()
{
  
  for (int i=0; i<4; i++)
  {
    outgoing.numbers[i] = x;
  }

  Wire.beginTransmission(69); // transmit to device 69
  Wire.write(outgoing.bytes, 16);
  Wire.endTransmission();

  Wire.requestFrom(69, 16);
  int ctr = 0;
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read();
    incoming.bytes[ctr]=c;
    if(ctr>=15) break;
    ctr++;
  }
  
  x = x + 0.125;
  delay(500);
}
