
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "CytronMotorDriver.h"

// Configure the motor driver.
CytronMD lmotor(PWM_DIR, 5, 26);  // PWM = Pin 9, DIR = Pin 22.
CytronMD rmotor(PWM_DIR, 4, 28);  // PWM = Pin 8, DIR = Pin 23.

const int motor_rpm = 75 ;
const float wheel_diameter = 0.081 ;
const float wheel_separation = 0.242 ;
const int max_pwm_val = 255 ;
const int min_pwm_val = 0 ;

const float wheel_radius = wheel_diameter / 2 ;
const float circumference_of_wheel = 2 * 3.145 * wheel_radius ;
const float max_speed = (circumference_of_wheel * motor_rpm ) / 60 ;
 
void onTwist(const geometry_msgs::Twist& msg)
{
  float linear_vel = msg.linear.x;
  float angular_vel = msg.angular.z;

  float VrplusVl  = 2 * linear_vel ;
  float VrminusVl = angular_vel * wheel_separation ;

  float right_vel = (  VrplusVl + VrminusVl ) / 2  ;
  float left_vel  = VrplusVl - right_vel    ;

  float absleft_vel = abs (left_vel) ;
  float absright_vel = abs (right_vel) ; 

  int lspeedPWM =   max ( min ( ( ( absleft_vel/max_speed)*max_pwm_val ) , max_pwm_val ) , min_pwm_val ) ;
  int rspeedPWM =   max ( min ( ( ( absright_vel/max_speed)*max_pwm_val ) , max_pwm_val ) , min_pwm_val ) ;  
  
  if ( left_vel >= 0 )
  {
    lmotor.setSpeed(lspeedPWM);
  }
  else
  {
    lmotor.setSpeed(-(lspeedPWM));
  }

  if ( right_vel >= 0 )
  {
    rmotor.setSpeed(-(rspeedPWM));
  }
  else
  {
    rmotor.setSpeed(rspeedPWM);
  }
  

}
 
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel",onTwist); 
 
ros::NodeHandle nh;
 
void setup() {
  // put your setup code here, to run once:
//  pinMode(lPWM,OUTPUT);
//  pinMode(rPWM,OUTPUT);
//  pinMode(lDir,OUTPUT);
//  pinMode(rDir,OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
