// Include Files
#include <Wire.h>
#include <MsTimer2.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
#include <math.h>
#include "DigitalTF.h"
#include "EncoderI2C.h"

// Macros
#define SIGN(n) (n<0 ? -1 : 1)  // sign() macro (-1 if negative, +1 if zero or positive)

SoftwareSerial BTserial(10, 13); // RX | TX
// Connect the HC-06 TX to the Arduino RX on pin 10. 
// Connect the HC-06 RX to the Arduino TX on pin 13.

// TB6612FNG Motor Driver Pin Definitions
#define DIR_L 7   // left motor direction (0 - forward, 1 - reverse)
#define DIR_R 12  // right motor direction (0 - forward, 1 - reverse)
#define PWM_L 5   // left motor PWM
#define PWM_R 6   // right motor PWM
#define STBY 8    // standby status (0 - disable, 1 - enable)

#define PIC_MCLR A3 // PIC reset pin (active-low)
#define PI_READY 11 // Raspberry Pi ready/busy signal

// IMU Variables
MPU6050 mpu;        // MPU6050 object
int16_t ax, ay, az; // IMU accelerations (FSR +-{2, 4, 8, 16} g, {16384, 8192, 4096, 2048} LSB/g)
int16_t wx, wy, wz; // IMU angular velocities (FSR +-{250, 500, 1000, 2000} deg/s, {131, 65.5, 32.75, 16.375} LSB/(deg/s))

// IMU Callibration Variables
const int16_t ax_cal = 233, ay_cal = -718, az_cal = 16464 - 16384;  // Obtained by callibrating accelerometer values (z-direction compensates for gravity)
const int16_t wx_cal = 407, wy_cal = 8, wz_cal = 163;               // Obtained by callibrating gyroscope values

// Encoder Variables
volatile long left_encoder = 0;
volatile long right_encoder = 0;

const uint16_t pulse_per_rev = 780 * 2;                       // encoder ticks per wheel revolution
const float millirev_per_pulse = 1000.0 / pulse_per_rev;  // milli-revolutions per encoder tick

// PIC Encoder Objects
const uint8_t left_enc_addr = 0x08;   // left I2C address
const uint8_t right_enc_addr = 0x09;  // right I2C address
EncoderI2C left_enc_obj = EncoderI2C(left_enc_addr);
EncoderI2C right_enc_obj = EncoderI2C(right_enc_addr);

// Motor PWM Input Values
int16_t motor_left = 0;  // (signed) PWM signal to left motor
int16_t motor_right = 0; // (signed) PWM signal to right motor

// Time variables
unsigned long time_curr;            // current time
unsigned long time_prev = 0;        // previous loop time
unsigned long startTime_left = 0;
unsigned long startTime_right = 0;
const int sampling_rate = 50;        // main loop period (ms)
const float sampling_rate_s = sampling_rate / 1000.0; // main loop period (s)

// Speed variables
//int encoder_count_max = 20;
//const int encoder_max_time = 5;
const int encoder_max_time = 0;
float motor_left_ang_vel = 0;
float motor_right_ang_vel = 0;

// Physical Dimensions
const float wheel_rad = 0.0335; // wheel radius (m)
const float lat_dist = 0.194;   // end-to-end wheel distance (m)

// Tilt Angle Estimation
const float alpha = 0.97;           // complementary filter coefficient
const float c_alpha = 1.0 - alpha;  // complementary filter coefficient complement

float tilt_ang = 0; // tilt angle estimate

// Tilt Angular Velocity Estimation
float tilt_vel = 0; // tilt angular velocity estimate using direct gyroscope measurement

// Robot Position Estimation
float pos = 0;
float vel = 0;

// LQR Controller Parameters
float K[] = {-0.70710678, -30.217762, 6330.7164, 124.35118};

const int16_t ctlr_LB = -255; // controller saturation lower bound
const int16_t ctlr_UB = 255;  // controller saturation upper bound

int16_t left_out = 0;
int16_t right_out = 0;
int16_t control_effort = 0;
int16_t control_effort_pre = 0;

// Raspberry Pi Interfacing
const uint8_t rpi_addr = 69;   // Raspberry Pi I2C address

// Union for receiving controller gains
typedef union
{
  float data[4];
  byte bytes[16];
} REC_UNION;

REC_UNION incoming; // receive union

// Structure to store mix of states (floats) and controls (ints)
typedef struct {
  float state[3];
  int16_t ctrl;
} SEND_STRUCT;

// Union for sending states and control
typedef union {
  SEND_STRUCT data;
  byte bytes[14];
} SEND_UNION;

SEND_UNION outgoing;  // send union

// Right Wheel Dynamics Matching
DigitalTF<int16_t, int16_t> rw_comp;
// 5 ms sampling period
const float comp_B[] = {0.35605129, -0.32377994};
const float comp_A[] = {1.0, -0.96314955};

/********************Initialization settings********************/
void setup() {
  // Motor driver control signal initialization
  pinMode(DIR_L, OUTPUT); // set left motor direction as output
  pinMode(DIR_R, OUTPUT); // set right motor direction as output
  pinMode(PWM_L, OUTPUT); // set left motor PWM as output
  pinMode(PWM_R, OUTPUT); // set right motor PWM as output
  pinMode(STBY, OUTPUT);  // set standby signal as output

  pinMode(PIC_MCLR, OUTPUT);
  pinMode(PI_READY, INPUT);

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
  Wire.begin();           // open I2C bus
  Serial.begin(57600);    // open serial port
  BTserial.begin(57600);  // open software-serial port (Bluetooth)
  delay(1500);            // buffer 1.5s

  // Initialize IMU
  mpu.initialize(); // initialization MPU6050
  delay(2);         // buffer 2 ms

  // Initialize right wheel compensator
  rw_comp.init(comp_B, sizeof(comp_B)/sizeof(float), comp_A, sizeof(comp_A)/sizeof(float));

  // IMU calibration
//  BTserial.println("Starting Delay");
//  delay(10000);
//  BTserial.println("Starting Calibration");
//  callibrateAccelValues();
//  callibrateGyroValues();

  // Store start times
  time_curr = millis();
  startTime_left = millis();
  startTime_right = millis();
  
  // Initialize main loop timer
  MsTimer2::set(sampling_rate, mainfunc);

  Serial.println("Setup Done!");  // print to console

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
   readIMU() creates an MPU6050 class object and calls the function to read the six axis IMU.
   The values are stored in the global variables ax,ay,az,wx,wy,wz where ax,ay,az are the
   accelerometer readings and wx,wy,wz are the gyroscope readings.
*/
void readIMU() {
//  MPU6050 mpu_obj;
//  mpu_obj.getMotion6(&ax, &ay, &az, &wx, &wy, &wz);
  mpu.getMotion6(&ax, &ay, &az, &wx, &wy, &wz);
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

/*
    printIMU() prints the IMU readings to the serial monitor in the following format:
    ax,ay,az,wx,wy,wz
*/
void printIMU() {
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(wx);
  Serial.print(',');
  Serial.print(wy);
  Serial.print(',');
  Serial.print(wz);
  Serial.print('\r');
  Serial.print('\n');
}

/*
    printEncoder() prints the encoder readings to the serial monitor in the following format:
    motor_left_ang_vel, motor_right_ang_vel
*/
void printEncoder() {
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print('\n');
}

/*
    printAllData() prints the IMU readings, encoder readings and PWM inputs to the motor to
    the serial monitor in the following format:
    ax,ay,az,wx,wy,wz,motor_left_ang_vel,motor_right_ang_vel,motor_left,motor_right
*/
void printAllData() {
  Serial.print(time_curr);
  Serial.print(',');
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(wx);
  Serial.print(',');
  Serial.print(wy);
  Serial.print(',');
  Serial.print(wz);
  Serial.print(',');
  Serial.print(motor_left_ang_vel);
  Serial.print(',');
  Serial.print(motor_right_ang_vel);
  Serial.print(',');
  Serial.print(motor_left);
  Serial.print(',');
  Serial.print(motor_right);
  Serial.print('\r');
  Serial.print('\n');
}

/*
    printMyData() prints whatever I want
*/
void printMyData() {
  BTserial.print(time_curr);
  BTserial.print(',');
  BTserial.print(tilt_ang, 4);
  BTserial.print(',');
  BTserial.print(tilt_vel, 3);
  BTserial.print(',');
  BTserial.print(motor_left);
  BTserial.print('\r');
  BTserial.print('\n');
}

/*
   callibrateAccelValues() gets the accelerometer readings n times and calculates the average of
   the values to find the sensor bias. The callibration values are printed as:
   accelXCalli,accelYCalli,accelZCalli
*/
void callibrateAccelValues() {
  long accelXCalli = 0, accelYCalli = 0, accelZCalli = 0;
  int n = 10000;
  for (int i = 0; i < n; i++) {
    readIMU();
    accelXCalli = accelXCalli + ax;
    accelYCalli = accelYCalli + ay;
    accelZCalli = accelZCalli + az;
  }
  accelXCalli = accelXCalli / n;
  accelYCalli = accelYCalli / n;
  accelZCalli = accelZCalli / n;
  Serial.print(accelXCalli);
  Serial.print(',');
  Serial.print(accelYCalli);
  Serial.print(',');
  Serial.println(accelZCalli);
  BTserial.print(accelXCalli);
  BTserial.print(',');
  BTserial.print(accelYCalli);
  BTserial.print(',');
  BTserial.println(accelZCalli);
}

/*
   callibrateGyroValues() gets the gyroscope readings n times and calculates the average of
   the values to find the sensor bias. The callibration values are printed as:
   gyroXCalli,gyroYCalli,gyroZCalli
*/
void callibrateGyroValues() {
  long gyroXCalli = 0, gyroYCalli = 0, gyroZCalli = 0;
  int n = 10000;
  for (int i = 0; i < n; i++) {
    readIMU();
    gyroXCalli = gyroXCalli + wx;
    gyroYCalli = gyroYCalli + wy;
    gyroZCalli = gyroZCalli + wz;
  }
  gyroXCalli = gyroXCalli / n;
  gyroYCalli = gyroYCalli / n;
  gyroZCalli = gyroZCalli / n;
//  Serial.print(gyroXCalli);
//  Serial.print(',');
//  Serial.print(gyroYCalli);
//  Serial.print(',');
//  Serial.println(gyroZCalli);
  BTserial.print(gyroXCalli);
  BTserial.print(',');
  BTserial.print(gyroYCalli);
  BTserial.print(',');
  BTserial.println(gyroZCalli);
}

/***************************************************************************************/
/***************** Write your custom variables and functions below *********************/
/***************************************************************************************/

/*
 * estimateTilt() estimates the robot tilt angle using a complementary filter to combine
 * the accelerometer and gyroscope measurements.
 */
void estimateTilt() {
  float Ax = (ax - ax_cal) / 16384.0;
  float Ay = (ay - ay_cal) / 16384.0;
  float Az = (az - az_cal) / 16384.0;
  float Wx = (wx - wx_cal) / 131.0 * DEG_TO_RAD;

  float acc_angle = atan2(Ay, Az);
  tilt_ang = alpha * (tilt_ang + Wx*sampling_rate_s) + c_alpha * acc_angle;
}

/*
 * estimateTiltVel() estimates the robot tilt angular velocity using the direct
 * gyroscope measurement.
 */
void estimateTiltVel() {
  tilt_vel = (wx - wx_cal) / 131.0 * DEG_TO_RAD;
}

void estimatePosition() {
  vel = wheel_rad * motor_left_ang_vel;
  pos += vel * sampling_rate_s;
}

void updateCtlr() {
  float outf = K[0]*pos + K[1]*vel + K[2]*tilt_ang + K[3]*tilt_vel;
  int16_t out = (int16_t) (outf + SIGN(outf)*0.5);
  control_effort = min(max(out, ctlr_LB), ctlr_UB);
}

void sendToPi() {
  while(!digitalRead(PI_READY));  // wait for RPi to prepare gains
  
  outgoing.data.state[0] = vel;
  outgoing.data.state[1] = tilt_ang;
  outgoing.data.state[2] = tilt_vel;
//  outgoing.data.ctrl = control_effort;
  outgoing.data.ctrl = control_effort_pre;
  
  Wire.beginTransmission(rpi_addr);
  Wire.write(outgoing.bytes, 14);
  Wire.endTransmission();
}

void readFromPi() {
  while(!digitalRead(PI_READY));  // wait for RPi to prepare gains

  Wire.requestFrom(rpi_addr, 16);
  size_t ctr = 0;
  while (Wire.available()) { // slave may send less than requested
    byte c = Wire.read();
    incoming.bytes[ctr++] = c;
    if(ctr >= 16) break;
  }

  K[0] = incoming.data[0];
  K[1] = incoming.data[1];
  K[2] = incoming.data[2];
  K[3] = incoming.data[3];
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
  readIMU();  // update IMU data
  /* Do not modify ends*/
  
  /*Write your code below*/
  estimateTilt();
  estimateTiltVel();
  estimatePosition();

  readFromPi();

  control_effort_pre = control_effort;
  updateCtlr();
  left_out = control_effort;
  right_out = rw_comp.update(left_out);
  
//  SetLeftWheelSpeed(left_out);
//  SetRightWheelSpeed(right_out);

  sendToPi();
  
//  printMyData();
  
//  Serial.println("Printing through USB Serial Port");
//  BTserial.println("Printing through Bluetooth");
  /***********************/
}

void loop() {

}
