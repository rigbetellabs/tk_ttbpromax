#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <Arduino.h>


#define DEBUG 0

#define left_pwm_pin 25
#define left_dir_pin 26

#define right_pwm_pin 32 
#define right_dir_pin 33 


rcl_subscription_t lpwm_subscriber;
rcl_subscription_t rpwm_subscriber;
rcl_subscription_t ldir_subscriber;
rcl_subscription_t rdir_subscriber;

std_msgs__msg__Int32 int_msg;
std_msgs__msg__Bool bool_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn;}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}




void ldirCallback (const void * msgin)
{
  const std_msgs__msg__Bool* leftdir = (const std_msgs__msg__Bool *)msgin;
  bool ldir = leftdir->data ;
  digitalWrite(left_dir_pin, ldir);
}

void rdirCallback (const void * msgin)
{
  const std_msgs__msg__Bool* rightdir = (const std_msgs__msg__Bool *)msgin;
  bool rdir = rightdir->data ;
  digitalWrite(right_dir_pin, rdir);

}
void lpwmCallback (const void * msgin)
{
  const std_msgs__msg__Int32* leftpwm = (const std_msgs__msg__Int32 *)msgin;
  int lpwm = leftpwm->data ;
  analogWrite(left_pwm_pin, lpwm);
}

void rpwmCallback (const void * msgin)
{
  const std_msgs__msg__Int32* rightpwm = (const std_msgs__msg__Int32 *)msgin;
  int rpwm = rightpwm->data ;
  analogWrite(right_pwm_pin, rpwm);
}


void setup() {
  set_microros_transports();


  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&lpwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                         "lpwm"));
  RCCHECK(rclc_subscription_init_default(&rpwm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                                         "rpwm"));
  RCCHECK(rclc_subscription_init_default(&ldir_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                         "ldir"));
  RCCHECK(rclc_subscription_init_default(&rdir_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                                         "rdir"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &lpwm_subscriber, &int_msg, &lpwmCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &rpwm_subscriber, &int_msg, &rpwmCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &ldir_subscriber, &bool_msg, &ldirCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &rdir_subscriber, &bool_msg, &rdirCallback, ON_NEW_DATA));

  pinMode (left_dir_pin, OUTPUT);
  pinMode (left_pwm_pin, OUTPUT);

  pinMode (right_pwm_pin, OUTPUT);
  pinMode (right_dir_pin, OUTPUT);
  pinMode (36, INPUT);
  pinMode (35, INPUT);
  pinMode (34, INPUT);
  pinMode (39, INPUT);
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
