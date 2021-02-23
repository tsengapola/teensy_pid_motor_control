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


struct pose_3d{
  double x,y,z;
  double roll,pitch,yaw;
  double vx;
  double ax,ay,az;
};

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

pose_3d pose_3d_ = {0,0,0,0,0,0,0,0,0,0};
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;

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

ros::Publisher odom_pub ("odom", &odom_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", &cmdCb );


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
  nh.advertise(odom_pub);

  //Start status led
  digitalWrite(statusPin, HIGH);

}

void computeOdom(pose_3d& m_pose_3d, double rpm_r, double rpm_l, sensors_event_t& gyro){

  double ax = gyro.gyro.x*0.017453293;
  double ay = gyro.gyro.y*0.017453293;
  double az = gyro.gyro.z*0.017453293;
  if(fabs(ax)<0.03)
    ax = 0.0;
  if(fabs(ay)<0.03)
    ay = 0.0;
  if(fabs(az)<0.03)
    az = 0.0;
    
  double dr = (m_pose_3d.ax+ax)/2.*loop_time_sec;
  double dp = (m_pose_3d.ay+ay)/2.*loop_time_sec;
  double dy = (m_pose_3d.az+az)/2.*loop_time_sec;
  
  double v_r = rpm_r/60.0/gear_ratio * 3.1415926535*wheel_diameter;
  double v_l = rpm_l/60.0/gear_ratio * 3.1415926535*wheel_diameter;
  double v = (v_r+v_l)/2.;

  double base_z_vel = (m_pose_3d.vx+v)/2. * -sin(m_pose_3d.pitch);
  double proj_z_vel = (m_pose_3d.vx+v)/2. * cos(m_pose_3d.pitch);
  double base_x_vel = proj_z_vel * cos( dy );
  double base_y_vel = proj_z_vel * sin( dy );

  
  double dt_odom_x = ( base_x_vel * cos(m_pose_3d.yaw) - base_y_vel * sin(m_pose_3d.yaw) ) * loop_time_sec;
  double dt_odom_y = ( base_x_vel * sin(m_pose_3d.yaw) + base_y_vel * cos(m_pose_3d.yaw) ) * loop_time_sec;
  double dt_odom_z = base_z_vel * loop_time_sec;
  
  m_pose_3d.x += dt_odom_x;
  m_pose_3d.y += dt_odom_y;
  m_pose_3d.z += dt_odom_z;
  m_pose_3d.yaw += dy;
  m_pose_3d.pitch += dp;
  m_pose_3d.roll += dr;
  m_pose_3d.vx = v;
  m_pose_3d.ax = ax;
  m_pose_3d.ay = ay;
  m_pose_3d.az = az;

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
    pose_3d_ = {0,0,0,0,0,0,0,0,0,0};
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
  //computeOdom(pose_3d_, measured_rpm, measured_rpm2, eulerData); //dps to rps
  computeOdom(pose_3d_, 300, 300, angVelocityData); //dps to rps
  
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = pose_3d_.x; 
  t.transform.translation.y = pose_3d_.y; 
  t.transform.translation.z = pose_3d_.z;
  t.transform.rotation = tf::createQuaternionFromRPY(pose_3d_.roll,pose_3d_.pitch,pose_3d_.yaw);
  t.header.stamp = nh.now();

  odom_msg.header = t.header;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = pose_3d_.x; 
  odom_msg.pose.pose.position.y = pose_3d_.y; 
  odom_msg.pose.pose.position.z = pose_3d_.z;
  odom_msg.pose.pose.orientation = t.transform.rotation;
  odom_msg.twist.twist.linear.x = pose_3d_.vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.x = pose_3d_.ax;
  odom_msg.twist.twist.angular.y = pose_3d_.ay;
  odom_msg.twist.twist.angular.z = pose_3d_.az;
    

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
