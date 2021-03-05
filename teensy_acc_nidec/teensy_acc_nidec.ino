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
#include <sensor_msgs/Imu.h>   
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

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
sensor_msgs::Imu imu_msg;

tf::TransformBroadcaster broadcaster;

void cmdCb( const geometry_msgs::Twist& cmd_msg){

  target_rpm = (cmd_msg.linear.x+base_radius*cmd_msg.angular.z) * 60 *gear_ratio/3.1415926535/wheel_diameter;
  target_rpm2 = (cmd_msg.linear.x-base_radius*cmd_msg.angular.z) * 60 *gear_ratio/3.1415926535/wheel_diameter;
  if(abs(target_rpm)<100)
    target_rpm = 0;
  if(abs(target_rpm2)<100)
    target_rpm2 = 0;
  HB = millis();
}

void joyCb( const sensor_msgs::Joy& joy_msg){

  //@ interlock
  
  if(joy_msg.axes[5]>=0.6){
    target_rpm = 0.5 * 60 *gear_ratio/3.1415926535/wheel_diameter;
    target_rpm2 = 0.5 * 60 *gear_ratio/3.1415926535/wheel_diameter;
    if(abs(target_rpm)<100)
      target_rpm = 0;
    if(abs(target_rpm2)<100)
      target_rpm2 = 0;
  }
  else if(joy_msg.axes[5]<=-0.6){
    target_rpm = -0.5 * 60 *gear_ratio/3.1415926535/wheel_diameter;
    target_rpm2 = -0.5 * 60 *gear_ratio/3.1415926535/wheel_diameter;
    if(abs(target_rpm)<100)
      target_rpm = 0;
    if(abs(target_rpm2)<100)
      target_rpm2 = 0;
  }

  HB = millis();
}

ros::Publisher odom_pub ("odom", &odom_msg);
ros::Publisher imu_pub ("imu", &imu_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdCb );
ros::Subscriber<sensor_msgs::Joy> joy_sub("joy", &joyCb );

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

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(cmd_sub);
  nh.subscribe(joy_sub);
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  //Start status led
  digitalWrite(statusPin, HIGH);

}

void computeOdom(pose_2d& m_pose_2d, double rpm_r, double rpm_l, double vz){
  
  if(abs(vz)<0.03)
    vz = 0.0;
    
  double dt_theta = (m_pose_2d.vz+vz)/2.*loop_time_sec;
  double v_r = rpm_r/60.0/gear_ratio * 3.1415926535*wheel_diameter;
  double v_l = rpm_l/60.0/gear_ratio * 3.1415926535*wheel_diameter;
  double v = (v_r+v_l)/2.;

  double base_x_vel = (m_pose_2d.v+v)/2. * cos( dt_theta );
  double base_y_vel = (m_pose_2d.v+v)/2. * sin( dt_theta );

  
  double dt_odom_x = ( base_x_vel * cos(m_pose_2d.theta) - base_y_vel * sin(m_pose_2d.theta) ) * loop_time_sec;
  double dt_odom_y = ( base_x_vel * sin(m_pose_2d.theta) + base_y_vel * cos(m_pose_2d.theta) ) * loop_time_sec;

  m_pose_2d.x += dt_odom_x;
  m_pose_2d.y += dt_odom_y;
  m_pose_2d.theta += dt_theta;
  m_pose_2d.v = v;
  m_pose_2d.vz = vz;

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
  
  if(abs(millis()-HB)>700){
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
  
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* put rpm of wheels and gyro by rps*/
  computeOdom(pose_2d_, measured_rpm, measured_rpm2, angVelocityData.gyro.z*0.017453293); //dps to rps

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = pose_2d_.x; 
  t.transform.translation.y = pose_2d_.y; 
  t.transform.translation.z = 0.0; 
  t.transform.rotation = tf::createQuaternionFromYaw(pose_2d_.theta);
  t.header.stamp = nh.now();

  odom_msg.header = t.header;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = pose_2d_.x; 
  odom_msg.pose.pose.position.y = pose_2d_.y; 
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = t.transform.rotation;
  odom_msg.twist.twist.linear.x = pose_2d_.v;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = pose_2d_.vz;

  /*Querying data from BNO055*/
  sensors_event_t accData;
  /* option: VECTOR_ACCELEROMETER/VECTOR_GRAVITY */
  bno.getEvent(&accData,Adafruit_BNO055::VECTOR_LINEARACCEL);

  imu_msg.header = odom_msg.header;
  imu_msg.linear_acceleration.x = accData.acceleration.x;
  imu_msg.linear_acceleration.y = accData.acceleration.y;
  imu_msg.linear_acceleration.z = accData.acceleration.z;

  broadcaster.sendTransform(t);
  odom_pub.publish(&odom_msg);
  imu_pub.publish(&imu_msg);
  
  nh.spinOnce();
  loop_cnt++;
  if(loop_cnt>=loop_cnt_sec){
    loop_cnt=0;
    digitalWrite(statusPin, !digitalRead(statusPin));
  }
  double cal_time = abs(millis()-calc_t);
  if(cal_time<=10)
    delay(loop_time-cal_time);
  else
    delay(loop_time);
  
}
