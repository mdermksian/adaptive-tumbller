#include <Wire.h>

#define PI_READY 11

typedef union
{
  float data[4];
  byte bytes[16];
} REC_UNION;

REC_UNION incoming;

typedef struct {
  float state[3];
  int16_t ctrl;
} SEND_STRUCT;

typedef union {
  SEND_STRUCT data;
  byte bytes[14];
} SEND_UNION;

SEND_UNION outgoing;

float x = 1.0f;

void setup() {
  pinMode(PI_READY, INPUT);
  Wire.begin(); // join i2c bus as master
  Serial.begin(57600);
}

void loop() {
  
  for (int i=0; i<3; i++) {
    outgoing.data.state[i] = x;
  }
  outgoing.data.ctrl = -220;

  Wire.beginTransmission(69); // transmit to device 69
  Wire.write(outgoing.bytes, 14);
  Wire.endTransmission();

  while(!digitalRead(PI_READY));  // wait for RPi to prepare gains

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
