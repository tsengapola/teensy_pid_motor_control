#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#include <ros.h>
#include <ros/time.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>   
#include <geometry_msgs/Twist.h>

BNO080 myIMU;

Encoder motor1(0, 1);
Encoder motor2(22, 23);

double measured_rpm;
double measured_rpm2;

const int PinA =  16;
const int PinB =  15;
const int PinC =  17;
const int PinD =  14;
const int servoPin = 5;

int pwmPin = 20;
int pwmPin2 = 21;

/*
 * PID
 */
double kp = 0.02;
double ki = 0.09;
double kd = 0.00022;

// ------- 1st PID -------
double error=0;
double lastError=0;
double target_rpm = 0;
double cumError = 0, rateError = 0;
double out = 0;

// ------- 2nd PID -------
double error2=0;
double lastError2=0;
double target_rpm2 = 0;
double cumError2 = 0, rateError2 = 0;
double out2 = 0;

/*
 * Loop time
 */
double loop_time = 40;
double loop_time_sec = loop_time/1000.;

/*
 * Servo
 */
double servo_ang_mapping = 75./1.570796327;
double anawrite = 0;
double servo_nc = 291; //nc point 145-->75 mapping to 90 deg
int servo_angle_mapping = 291;


void setup() {

  //Serial.begin(115200);
  
  pinMode(PinA, OUTPUT);
  pinMode(PinB, OUTPUT);
  pinMode(PinC, OUTPUT);
  pinMode(PinD, OUTPUT);
  pinMode(pwmPin,OUTPUT); 
  pinMode(pwmPin2,OUTPUT);
  pinMode(servoPin,OUTPUT); 
  analogWriteResolution(9);
  analogWriteFrequency(pwmPin, 93750); 
  analogWriteFrequency(pwmPin2, 93750); 
  analogWriteFrequency(servoPin, 488); 

  

}


void loop() {
  
  analogWrite(servoPin, 291+150);

}
