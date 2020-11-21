// Include Files
// You'll need the EncoderI2C folder wherever your Arduino libraries are stored in.
#include "EncoderI2C.h"

// PIC reset pin (active-low)
// Set to 0 briefly in setup, then return to 1. Just resets the PIC microcontrollers.
#define PIC_MCLR A3

// PIC I2C addresses
// Used to select which PIC to communicate with.
// THESE ARE FIXED, DON'T CHANGE THEM
const uint8_t left_enc_addr = 0x08;   // left I2C address
const uint8_t right_enc_addr = 0x09;  // right I2C address

// PIC Encoder class instances
// Should make 2, one for each motor.
EncoderI2C left_enc_obj = EncoderI2C(left_enc_addr);
EncoderI2C right_enc_obj = EncoderI2C(right_enc_addr);

// Encoder Resolution
// With PIC microcontroller, this number is double what it used to be.
const float num_of_encoder_counts_per_rev = 1560.0;                                             // encoder ticks per wheel revolution
const float thousand_by_num_of_encoder_counts_per_rev = 1000.0 / num_of_encoder_counts_per_rev; // milli-revolutions per encoder tick

// Encoder count variables
// You can keep using these to store the velocity in tick-units if you want. But here they aren't used.
volatile long left_encoder;
volatile long right_encoder;

// Encoder update period
// Just set this to zero, otherwise might have weird "skips".
const int encoder_count_max = 0;

// These should already exist
unsigned long time;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;


void setup() {
  // Reset PICs
  pinMode(PIC_MCLR, OUTPUT);
  digitalWrite(PIC_MCLR, 0);
  delay(10);  // Give enough time for PICs to reset (probably more than needed)
  digitalWrite(PIC_MCLR, 1);
  delay(100); // Give enough time for PICs to go through their own initializations (probably more than needed)

  // You should GET RID OF the pin change interrupt lines since the encoders aren't being read that way anymore.

  Serial.begin(57600);
}


// You can also get rid of the encoder_left() & encoder_right() functions if you want, but removing the setup portion is enough.


// Replace readEncoder() with this function that utilizes the PICs
void readEncoder() {
  // Reads the counts over I2C and updates member variables
  left_enc_obj.readEncoder();
  right_enc_obj.readEncoder();

  // Compute left angular velocity
  motor_left_ang_vel = TWO_PI * left_enc_obj.getChange() * thousand_by_num_of_encoder_counts_per_rev / (time - startTime_left); // calculate angular velocity
  startTime_left = time; // set new encoder start time
  
  // Compute right angular velocity
  motor_right_ang_vel = TWO_PI * right_enc_obj.getChange() * thousand_by_num_of_encoder_counts_per_rev / (time - startTime_right);
  startTime_right = time;
}


void loop() {
  delay(500); // fake sampling delay

  time = millis();
  readEncoder();

  Serial.print("Left:\t");
  Serial.print(left_enc_obj.getCount());  // absolute encoder position
  Serial.print('\t');
  Serial.print(left_enc_obj.getChange()); // change since last readEncoder()
  Serial.print('\t');
  Serial.println(motor_left_ang_vel); // actual velocity in rad/s
  
  Serial.print("Right:\t");
  Serial.print(right_enc_obj.getCount());  // absolute encoder position
  Serial.print('\t');
  Serial.print(right_enc_obj.getChange()); // change since last readEncoder()
  Serial.print('\t');
  Serial.println(motor_right_ang_vel); // actual velocity in rad/s
  Serial.println();
}
