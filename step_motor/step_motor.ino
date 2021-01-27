/*ROS*/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Joy.h>

int dirPin = 2;
int clkPin = 3;
const int statusPin = 13;


/*
 * Loop time
 */
double loop_time = 40;
double loop_time_sec = loop_time/1000.;
int loop_cnt_sec = 1000/loop_time;
int loop_cnt = 0;

/*
 * ROS
 */
ros::NodeHandle  nh;

/*False safe*/
unsigned long HB;

void joyCb( const sensor_msgs::Joy& joy_msg){

  //@ interlock
  if(joy_msg.buttons[7]==1){
    
    if(joy_msg.buttons[3]==1){
      digitalWrite(dirPin, HIGH);
      analogWrite(clkPin, 200);
      HB = millis();
    }
    else if(joy_msg.buttons[1]==1){
      digitalWrite(dirPin, LOW);
      analogWrite(clkPin, 200);
      HB = millis();
    }
    else{
      //@ stop motor
      analogWrite(clkPin, 0);
    }

    if(joy_msg.buttons[4]==1){
      analogWriteFrequency(clkPin, 4000); 
    }
    else{
      analogWriteFrequency(clkPin, 2000); 
    }
    
  }
  
  
}

ros::Subscriber<sensor_msgs::Joy> joy_sub("joy", &joyCb);

void setup() {
  pinMode(dirPin,OUTPUT); 
  pinMode(clkPin,OUTPUT);
  pinMode(statusPin,OUTPUT); 
  analogWriteResolution(8);
  analogWriteFrequency(clkPin, 1600); 

  digitalWrite(dirPin, HIGH);
  analogWrite(clkPin, 0);
  
  nh.initNode();
  nh.subscribe(joy_sub);
}

void loop() {

  if(!nh.connected())
  {
    analogWrite(clkPin, 0);
    loop_cnt_sec = 200/loop_time;
  }
  else{
    loop_cnt_sec = 1000/loop_time;
  }
  
  if(abs(millis()-HB)>500){
    analogWrite(clkPin, 0);
  }

  
  loop_cnt++;
  if(loop_cnt==loop_cnt_sec){
    loop_cnt=0;
    digitalWrite(statusPin, !digitalRead(statusPin));
  }

  nh.spinOnce();
  delay(loop_time);
}
