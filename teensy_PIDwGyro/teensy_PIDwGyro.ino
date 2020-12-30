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
float servo_ang_mapping = 151./1.570796327; //teensy use 9 bit pwm which is 150 equal 90 degree, where mega is 75=90degree
float servo_ang_mapping_negative = 145./1.570796327; //teensy use 9 bit pwm which is 150 equal 90 degree, where mega is 75=90degree
float anawrite = 0;
float servo_nc = 290; //nc point 145-->75 mapping to 90 deg
int servo_angle_mapping = 290;


/*
 * ROS
 */
ros::NodeHandle  nh;
char base_link[] = "/base_link";
char odom[] = "/odom";
char warn[] = "Alive";
char cbb[] = "cb";

geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;

tf::TransformBroadcaster broadcaster;

void cmdCb( const geometry_msgs::Twist& cmd_msg){
  target_rpm = cmd_msg.linear.x;
  target_rpm2 = cmd_msg.linear.y;
  if(cmd_msg.angular.z>0){
    anawrite = cmd_msg.angular.z * servo_ang_mapping + servo_nc;
    anawrite = min(anawrite,servo_nc+50);
  }
  else{
    anawrite = cmd_msg.angular.z * servo_ang_mapping_negative + servo_nc;   
    anawrite = max(anawrite,servo_nc-50);
  }
  servo_angle_mapping = (int)anawrite;

}

ros::Publisher odom_pub ("odom", &odom_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("rpm_vel", &cmdCb );


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
  
  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGameRotationVector(33); //Send data update every 33ms
  
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(cmd_sub);
  nh.advertise(odom_pub);
}


void loop() {
  
  while (!nh.connected())
  {
    target_rpm = 0.0;
    target_rpm2 = 0.0;
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, LOW);  
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, LOW);  
    nh.spinOnce();
  }
  //nh.logwarn(warn);
  long directionTick_1, directionTick_2;
  directionTick_1 = motor1.read();
  directionTick_2 = motor2.read();

  measured_rpm = directionTick_1/11.*60.*1000./loop_time/4.; //4 is mapping of teensy
  measured_rpm2 = directionTick_2/11.*60.*1000./loop_time/4.;
    
  motor1.write(0);
  motor2.write(0);

  //1st PID using AB
  error = target_rpm - measured_rpm;
  cumError += error * loop_time_sec;                // compute integral
  rateError = (error - lastError)/loop_time_sec;   // compute derivative
  out = kp*error + ki*cumError + kd*rateError;
  lastError = error; 
  /*
  If target is zero, lock it by driver API
  */
  
  if(abs(target_rpm)==0){
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, LOW);      
  }
  else if(out>0){
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, HIGH);
    out = min(out,510);   
  }
  else{
    digitalWrite(PinA, HIGH);
    digitalWrite(PinB, LOW);   
    out = out*-1.;      
    out = max(out,-510);  
  }
  analogWrite(pwmPin, out);
  
  //2nd PID using CD
  error2 = target_rpm2 - measured_rpm2;
  cumError2 += error2 * loop_time_sec;                // compute integral
  rateError2 = (error2 - lastError2)/loop_time_sec;   // compute derivative
  out2 = kp*error2 + ki*cumError2 + kd*rateError2;
  lastError2 = error2; 
  /*
  If target is zero, lock it by driver API
  */
  
  if(abs(target_rpm)==0){
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, LOW);      
  }
  else if(out2>0){
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, HIGH); 
    out2 = min(out2,510);       
  }
  else{
    digitalWrite(PinC, HIGH);
    digitalWrite(PinD, LOW); 
    out2 = out2*-1.;    
    out2 = max(out2,-510);     
  }
  analogWrite(pwmPin2, out2);

  /*
   * Get queternion from BNO080
   */
  
  float quatI,quatJ,quatK,quatReal;
  if (myIMU.dataAvailable() == true)
  {
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = 0.0; 
    t.transform.translation.y = 0.0; 
    t.transform.translation.z = 0.0; 
    t.transform.rotation.x = myIMU.getQuatI();
    t.transform.rotation.y = myIMU.getQuatJ(); 
    t.transform.rotation.z = myIMU.getQuatK(); 
    t.transform.rotation.w = myIMU.getQuatReal();
    t.header.stamp = nh.now();

    odom_msg.header = t.header;
    odom_msg.child_frame_id = base_link;
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = myIMU.getQuatI();
    odom_msg.pose.pose.orientation.y = myIMU.getQuatJ(); 
    odom_msg.pose.pose.orientation.z = myIMU.getQuatK(); 
    odom_msg.pose.pose.orientation.w = myIMU.getQuatReal();
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
  }
  else{
    delay(loop_time);
  }
  
  analogWrite(servoPin, servo_angle_mapping);
  //analogWrite(servoPin, servo_nc+151);
  broadcaster.sendTransform(t);
  odom_pub.publish(&odom_msg);
  
  nh.spinOnce();
  delay(loop_time);
  
}
