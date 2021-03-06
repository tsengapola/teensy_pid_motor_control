#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/*IMU*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*ROS*/
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>   
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

struct pose_2d{
  double x,y,theta;
  double v,vz;
} ;

/*Car kinematics*/
double base_radius = 0.165;
double wheel_diameter = 0.125;
double ppr = 100;
double gear_ratio = 32;

Encoder motor1(0, 1);
Encoder motor2(23, 22);

double measured_rpm;
double measured_rpm2;

const int PinA =  16;
const int PinB =  15;
const int PinC =  17;
const int PinD =  14;
const int statusPin = 13;

int pwmPin = 9;
int pwmPin2 = 10;

/*
 * PID
 */
double kp = 0.01;
double ki = 0.7;
double kd = 0.00002;

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
int loop_cnt_sec = 1000/loop_time;
int loop_cnt = 0;
/*IMU*/
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 20;

/*
 * ROS
 */
ros::NodeHandle  nh;
char base_link[] = "/base_link";
char odom[] = "/odom";
char warn[] = "Alive";
char cbb[] = "cb";

/*False safe*/
unsigned long HB;

pose_2d pose_2d_ = {0,0,0,0,0};
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;

tf::TransformBroadcaster broadcaster;

void joyCb( const sensor_msgs::Joy& joy_msg){

  //@ interlock
  
  if(joy_msg.buttons[7]==1){
    
    double dir=1.0;
    if(joy_msg.buttons[3]==1){
      HB = millis();
      target_rpm = -3000;
      target_rpm2 = -3000;
    }
    else if(joy_msg.buttons[1]==1){
      HB = millis();
      target_rpm = 3000;
      target_rpm2 = 3000;
    }
    else{
      target_rpm = 0;
      target_rpm2 = 0;
    }

    if(joy_msg.buttons[4]==1){
      target_rpm *= 2;
      target_rpm2 *= 2;
    }
    
  }
  
  
}

ros::Publisher odom_pub ("odom", &odom_msg);
ros::Subscriber<sensor_msgs::Joy> joy_sub("joy", &joyCb);


void setup() {

  //Serial.begin(115200);
  
  pinMode(PinA, OUTPUT);
  pinMode(PinB, OUTPUT);
  pinMode(PinC, OUTPUT);
  pinMode(PinD, OUTPUT);
  pinMode(pwmPin,OUTPUT); 
  pinMode(pwmPin2,OUTPUT);
  pinMode(statusPin,OUTPUT); 
  analogWriteResolution(9);
  analogWriteFrequency(pwmPin, 93750); 
  analogWriteFrequency(pwmPin2, 93750); 

  
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(joy_sub);
  nh.advertise(odom_pub);

  //Start status led
  digitalWrite(statusPin, HIGH);

}

void loop() {
  
  if(!nh.connected())
  {
    target_rpm = 0.0;
    target_rpm2 = 0.0;
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, LOW);  
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, LOW);  
    pose_2d_ = {0,0,0,0,0};
    loop_cnt_sec = 200/loop_time;
  }
  else{
    loop_cnt_sec = 1000/loop_time;
  }
  
  if(abs(millis()-HB)>300){
    target_rpm = 0.0;
    target_rpm2 = 0.0;
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, LOW);  
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, LOW);  
  }
  
  //nh.logwarn(warn);
  unsigned long calc_t = millis();
  long directionTick_1, directionTick_2;
  directionTick_1 = motor1.read();
  directionTick_2 = motor2.read();

  measured_rpm = directionTick_1/ppr*60.*1000./loop_time/4.; //4 is mapping for teensy/arduino
  measured_rpm2 = directionTick_2/ppr*60.*1000./loop_time/4.;
    
  motor1.write(0);
  motor2.write(0);

  //1st PID using AB
  error = target_rpm - measured_rpm;
  cumError += error * loop_time_sec;                // compute integral
  rateError = (error - lastError)/loop_time_sec;   // compute derivative
  double kp_adaptive = kp;
  out = kp_adaptive*error + ki*cumError + kd*rateError;
  lastError = error; 
  /*
  If target is zero, lock it by driver API
  */
 
  if(abs(target_rpm)<=50 && abs(target_rpm2)<=50 && abs(measured_rpm)<1000){
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, HIGH); 
  }
  else if(target_rpm>0){
    digitalWrite(PinA, HIGH);
    digitalWrite(PinB, HIGH);
    out = min(out,510);   
  }
  else{
    digitalWrite(PinA, HIGH);
    digitalWrite(PinB, LOW);   
    out = out*-1.;      
    out = min(out,510);  
  }
  analogWrite(pwmPin, 510-out);
  
  
  //2nd PID using CD
  error2 = target_rpm2 - measured_rpm2;
  cumError2 += error2 * loop_time_sec;                // compute integral
  rateError2 = (error2 - lastError2)/loop_time_sec;   // compute derivative
  double kp_adaptive2 = kp;
  out2 = kp_adaptive2*error2 + ki*cumError2 + kd*rateError2;
  lastError2 = error2; 
  /*
  If target is zero, lock it by driver API
  */
  
  if(abs(target_rpm)<=50 && abs(target_rpm2)<=50 && abs(measured_rpm2)<1000){
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, HIGH);      
  }
  else if(target_rpm2>0){
    digitalWrite(PinC, HIGH);
    digitalWrite(PinD, LOW); 
    out2 = min(out2,510);       
  }
  else{
    digitalWrite(PinC, HIGH);
    digitalWrite(PinD, HIGH); 
    out2 = out2*-1.;    
    out2 = min(out2,510);     
  }
  analogWrite(pwmPin2, 510-out2);

  /*
   * Get queternion from IMU
   */
  

  odom_msg.header = t.header;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = 0.0; 
  odom_msg.pose.pose.position.y = 0.0; 
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.twist.twist.linear.x = target_rpm;
  odom_msg.twist.twist.linear.y = target_rpm2;
  odom_msg.twist.twist.angular.x = measured_rpm;
  odom_msg.twist.twist.angular.y = measured_rpm2;
    

  broadcaster.sendTransform(t);
  odom_pub.publish(&odom_msg);
  
  nh.spinOnce();
  loop_cnt++;
  if(loop_cnt==loop_cnt_sec){
    loop_cnt=0;
    digitalWrite(statusPin, !digitalRead(statusPin));
  }
  double cal_time = abs(millis()-calc_t);
  if(cal_time<=10)
    delay(loop_time-cal_time);
  else
    delay(loop_time);
  
}
