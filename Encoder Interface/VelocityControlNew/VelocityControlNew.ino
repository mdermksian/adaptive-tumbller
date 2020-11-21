// Include Files
#include <PinChangeInt.h>
#include <MsTimer2.h>
#include "DigitalTF.h"
#include "EncoderI2C.h"

// TB6612FNG Motor Driver Pin Definitions
#define DIR_L 7   // left motor direction (0 - forward, 1 - reverse)
#define DIR_R 12  // right motor direction (0 - forward, 1 - reverse)
#define PWM_L 5   // left motor PWM
#define PWM_R 6   // right motor PWM
#define STBY 8    // standby status (0 - disable, 1 - enable)

#define PIC_MCLR A3 // PIC reset pin (active-low)

// PIC Encoder Objects
const uint8_t left_enc_addr = 0x08;   // left I2C address
const uint8_t right_enc_addr = 0x09;  // right I2C address
EncoderI2C left_enc_obj = EncoderI2C(left_enc_addr);
EncoderI2C right_enc_obj = EncoderI2C(right_enc_addr);

// Encoder Variables
volatile long left_encoder;
volatile long right_encoder;

const uint16_t pulse_per_rev = 780 * 2;                   // encoder ticks per wheel revolution
const float millirev_per_pulse = 1000.0 / pulse_per_rev;  // milli-revolutions per encoder tick

// Motor PWM Input Values
int16_t motor_left = 0;  // (signed) PWM signal to left motor
int16_t motor_right = 0; // (signed) PWM signal to right motor

// Time variables
unsigned long time_curr;            // current time
unsigned long time_prev = 0;        // previous loop time
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
const int sampling_rate = 5;        // main loop period (ms)

// Speed variables
//const int encoder_max_time = 20;
//const int encoder_max_time = 5;
const int encoder_max_time = 0;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

byte cmd[5];
uint8_t bytes_pend = 0;

// Physical Dimensions
const float wheel_rad = 0.0335; // wheel radius (m)
const float lat_dist = 0.194;   // end-to-end wheel distance (m)

// Controller Parameters
typedef float ctlr_in_t;    // variable type for controller input (i.e. error)
typedef int16_t ctlr_out_t; // variable type for controller output (i.e. control effort)

DigitalTF<ctlr_in_t, ctlr_out_t> ctlr;

// Left PID Auto
const float ctlr_B[] = {21.909140, -41.302216, 19.466157};  // numerator coefficients (b0*z^(m-n) + b1*z^(m-n-1) + ... + bm*z^(-n))
const float ctlr_A[] = {1.0, -1.9201780, 0.92017800};       // denominator coefficients (a0 + a1*z^(-1) + ... + an*z^(-n))
// Left PI Manual
//const float ctlr_B[] = {30.0, -28.873603};  // numerator coefficients
//const float ctlr_A[] = {1.0, -1.0};         // denominator coefficients
// Right PID Auto
//const float ctlr_B[] = {14.627264, -17.145846, 3.7491503};  // numerator coefficients
//const float ctlr_A[] = {1.0, -1.4116849, 0.41168488};       // denominator coefficients
// Right PI Manual
//const float ctlr_B[] = {30.0, -27.150974};  // numerator coefficients
//const float ctlr_A[] = {1.0, -1.0};         // denominator coefficients

const ctlr_out_t ctlr_LB = -255;  // controller saturation lower bound
const ctlr_out_t ctlr_UB = 255;   // controller saturation upper bound

volatile ctlr_in_t ref = 0.0;  // reference velocity (rad/s)


/********************Initialization settings********************/
void setup() {
  // Motor driver control signal initialization
  pinMode(DIR_L, OUTPUT); // set left motor direction as output
  pinMode(DIR_R, OUTPUT); // set right motor direction as output
  pinMode(PWM_L, OUTPUT); // set left motor PWM as output
  pinMode(PWM_R, OUTPUT); // set right motor PWM as output
  pinMode(STBY, OUTPUT);  // set standby signal as output
  
  pinMode(PIC_MCLR, OUTPUT);

  // Initializing motor drive module
  digitalWrite(DIR_L, 0); // left motor forward
  digitalWrite(DIR_R, 0); // right motor forward
  analogWrite(PWM_L, 0);  // left motor zero speed
  analogWrite(PWM_R, 0);  // right motor zero speed
  digitalWrite(STBY, 1);  // enable motor driver

  // Reset PICs
  digitalWrite(PIC_MCLR, 0);
  delay(10);
  digitalWrite(PIC_MCLR, 1);
  delay(100);

  // Initialize communications
  Wire.begin();         // open I2C bus
  Serial.begin(57600);  // open serial port
  delay(1500);          // buffer 1.5s

  // Initialize velocity controller
  ctlr.init(ctlr_B, sizeof(ctlr_B)/sizeof(float), ctlr_A, sizeof(ctlr_A)/sizeof(float), ctlr_LB, ctlr_UB);

  // Store start times
  time_curr = millis();
  startTime_left = millis();
  startTime_right = millis();

  // Initialize main loop timer
  MsTimer2::set(sampling_rate, mainfunc);

  Serial.print("Setup Done!\n");  // print to console

  // Start main loop timer
  MsTimer2::count = MsTimer2::msecs - 1;  // set so next tick triggers overflow
  MsTimer2::start();                      // start timer
}

/***************************************************************************************/
/*
  This section contains functions that you can use to build your controllers
*/
/***************************************************************************************/

/*
    SetLeftWheelSpeed() takes one input which will be set as the PWM input to the left motor.
    If the value is outside the range (-255,255), then the input will be saturated and the
    motor PWM will be set. The value is also written to the global variable 'motor_left'
*/
void SetLeftWheelSpeed(double speed_val) {
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) {
    speed_val = 255;
  } else if (speed_val < -255) {
    speed_val = -255;
  }

  motor_left = speed_val;

  if (speed_val < 0) {
    digitalWrite(DIR_L, 1);
    analogWrite(PWM_L, -speed_val);
  } else {
    digitalWrite(DIR_L, 0);
    analogWrite(PWM_L, speed_val);
  }
}

/*
    SetRightWheelSpeed() takes one input which will be set as the PWM input to the right motor.
    If the value is outside the range (-255,255), then the input will be saturated and the
    motor PWM will be set. The value is also written to the global variable 'motor_right'
*/
void SetRightWheelSpeed(double speed_val) {
  // Saturate the input to the range (-255,255)
  if (speed_val > 255) {
    speed_val = 255;
  } else if (speed_val < -255) {
    speed_val = -255;
  }

  motor_right = speed_val;

  if (speed_val < 0) {
    digitalWrite(DIR_R, 1);
    analogWrite(PWM_R, -speed_val);
  } else {
    digitalWrite(DIR_R, 0);
    analogWrite(PWM_R, speed_val);
  }
}

/*
   readEncoder() takes the encoder pulse counts and calculates the angular velocities of the
   wheels and stores it in the global variables 'motor_left_ang_vel' and 'motor_right_ang_vel'
*/
void readEncoder() {
  int32_t temp_cnt = 0;
  // Encoder Calculations
  // angular velocity = (encoder_reading/num_of_counts_per_rotation)*(2*pi/sampling_time)

  left_enc_obj.readEncoder();
  right_enc_obj.readEncoder();

  motor_left_ang_vel = TWO_PI * left_enc_obj.getChange() * millirev_per_pulse / (time_curr - startTime_left); // calculate angular velocity
  startTime_left = time_curr; // set new encoder start time

  motor_right_ang_vel = TWO_PI * right_enc_obj.getChange() * millirev_per_pulse / (time_curr - startTime_right);
  startTime_right = time_curr;
}


/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/

/*
    printMyData() prints whatever I want
*/
void printMyData() {
//  Serial.print(time_curr);
//  Serial.print(',');
  Serial.print(ref);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
//  Serial.print(',');
//  Serial.print(left_enc_obj.getCount());
//  Serial.print(',');
//  Serial.print(motor_left);
//  Serial.print(',');
//  Serial.print(motor_right_ang_vel);
//  Serial.print(',');
//  Serial.print(right_enc_obj.getCount());
//  Serial.print(',');
//  Serial.print(motor_right);
  Serial.print('\n');
  Serial.print('\r');
}

/*
   mainfunc() is the function that is called at your specified sampling rate. The default
   sampling rate is 5ms. This function will be called at every sampling instance.
*/
void mainfunc() {
  /* Do not modify begins*/
  sei();                // enable interrupts
  time_curr = millis(); // update current time
  if (time_curr - time_prev >= encoder_max_time) {
    readEncoder();          // update motor velocities
    time_prev = time_curr;  // store time
  }
  /* Do not modify ends*/

  /*Write your code below*/
  // Left Velocity Control
  ctlr.update(ref - motor_left_ang_vel);
  SetLeftWheelSpeed(ctlr.getOutput());

  // Right Velocity Control
//  ctlr.update(ref - motor_right_ang_vel);
//  SetRightWheelSpeed(ctlr.getOutput());

  printMyData();  // output wheel velocities over serial
}

void loop() {
  delay(500);

  // Step response inputs (only 1 at a time)
  ref = 8.0;
//  ref = -8.0;
  delay(2000);
  ref = 0.0;

  while (1);
}
