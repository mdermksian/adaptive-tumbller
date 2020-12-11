#include <Wire.h>

typedef union
{
  float numbers[4];
  byte bytes[16];
} FLOATUNION;

//FLOATUNION outgoing;
FLOATUNION incoming;

typedef struct {
  float state[4];
  int16_t ctrl;
} SENDTYPE;

typedef union {
  SENDTYPE data;
  byte bytes[18];
} SENDUNION;

SENDUNION outgoing;

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
    outgoing.data.state[i] = x;
  }
  outgoing.data.ctrl = -220;

  Wire.beginTransmission(69); // transmit to device 69
  Wire.write(outgoing.bytes, 18);
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
