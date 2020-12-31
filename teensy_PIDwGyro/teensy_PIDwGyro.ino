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


struct pose_2d{
  double x,y,theta;
  double v,vz;
} ;

/*Car kinematics*/
double base_radius = 0.3;
double wheel_diameter = 0.15;
double ppr = 100;

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

pose_2d pose_2d_ = {0,0,0,0,0};
geometry_msgs::TransformStamped t;
nav_msgs::Odometry odom_msg;

tf::TransformBroadcaster broadcaster;

void cmdCb( const geometry_msgs::Twist& cmd_msg){
  target_rpm = cmd_msg.linear.x;
  target_rpm2 = cmd_msg.linear.y;

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
}

void computeOdom(pose_2d& m_pose_2d, double rpm_r, double rpm_l, double vz){
  
  if(abs(vz)<0.03)
    vz = 0.0;
    
  double dt_theta = (m_pose_2d.vz+vz)/2.*loop_time_sec;
  double v_r = rpm_r/60.0 * 3.1415926535*wheel_diameter;
  double v_l = rpm_l/60.0 * 3.1415926535*wheel_diameter;
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
  
  while (!nh.connected())
  {
    target_rpm = 0.0;
    target_rpm2 = 0.0;
    digitalWrite(PinA, LOW);
    digitalWrite(PinB, LOW);  
    digitalWrite(PinC, LOW);
    digitalWrite(PinD, LOW);  
    pose_2d_ = {0,0,0,0,0};
    nh.spinOnce();
  }
  //nh.logwarn(warn);
  unsigned long calc_t = millis();
  long directionTick_1, directionTick_2;
  directionTick_1 = motor1.read();
  directionTick_2 = motor2.read();

  measured_rpm = directionTick_1/ppr*60.*1000./loop_time/4.; //4 is mapping of teensy
  measured_rpm2 = directionTick_2/ppr*60.*1000./loop_time/4.;
    
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
  odom_msg.twist.twist.angular.z = pose_2d_.vz;
    

  broadcaster.sendTransform(t);
  odom_pub.publish(&odom_msg);
  
  nh.spinOnce();
  double cal_time = abs(millis()-calc_t);
  if(cal_time<=10)
    delay(loop_time-cal_time);
  else
    delay(loop_time);
  
}
