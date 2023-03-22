#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include "esp_attr.h"

//Message types
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>



#define RCCHECK(fn) { rcl_ret_t temp_rc = fn;}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)\

#define left_pwm_pin  32
#define left_dir_pin 33

#define right_pwm_pin  25
#define right_dir_pin  26

#define rf_en_a 49
#define rf_en_b 36

#define lf_en_a 35
#define lf_en_b 34


    enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state;

  //Declare publishers and subscribers
  rcl_publisher_t publisher1;
  rcl_publisher_t publisher2;
  rcl_subscription_t lpwm_subscriber;
  rcl_subscription_t rpwm_subscriber;
  rcl_subscription_t ldir_subscriber;
  rcl_subscription_t rdir_subscriber;

  //message declaration
  std_msgs__msg__Int32 int_msg;
  std_msgs__msg__Bool bool_msg;
  std_msgs__msg__Int32 msg_r;
  std_msgs__msg__Int32 msg_l;

  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;

  // True = Forward; False = Reverse
  boolean Direction_left = true;
  boolean Direction_right = true;

  // Minumum and maximum values for 16-bit integers
  const int encoder_minimum = -32768;
  const int encoder_maximum =  32767;

  int rf_tick_count, rr_tick_count, lf_tick_count, lr_tick_count;
  bool ticks_reset_flag = false;
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

  bool create_entities() {

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

    // create publisher
    RCCHECK(rclc_publisher_init_default(
              &publisher1,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
              "left_ticks"));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
              &publisher2,
              &node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
              "right_ticks"));

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &ldir_subscriber, &bool_msg, &ldirCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &rdir_subscriber, &bool_msg, &rdirCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &lpwm_subscriber, &int_msg, &lpwmCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &rpwm_subscriber, &int_msg, &rpwmCallback, ON_NEW_DATA));

    return true;
  }

  void destroy_entities()
  {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&publisher1, &node);
    rcl_publisher_fini(&publisher2, &node);

    rcl_subscription_fini(&lpwm_subscriber, &node);
    rcl_subscription_fini(&rpwm_subscriber, &node);
    rcl_subscription_fini(&ldir_subscriber, &node);
    rcl_subscription_fini(&rdir_subscriber, &node);


    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
  }
  void setup() {
    set_microros_transports();
    state = WAITING_AGENT;
    Serial2.begin(115200);

    pinMode (left_dir_pin, OUTPUT);
    pinMode (left_pwm_pin, OUTPUT);

    pinMode (right_pwm_pin, OUTPUT);
    pinMode (right_dir_pin, OUTPUT);

    pinMode(rf_en_a, INPUT);
    pinMode(rf_en_b, INPUT);

    pinMode(lf_en_a, INPUT_PULLUP);
    pinMode(lf_en_b, INPUT_PULLUP);

    // Every time the pin goes high, this is a tick
    attachInterrupt(digitalPinToInterrupt(rf_en_a), rf_tick, RISING);
    attachInterrupt(digitalPinToInterrupt(lf_en_a), lf_tick, RISING);



  }


  // Increment the number of ticks
  void rf_tick() {

    // Read the value for the encoder for the right wheel
    int val = digitalRead(rf_en_b);

    if (val == LOW) {
      Direction_right = false;  // Reverse
    } else {
      Direction_right = true;  // Forward
    }

    if (Direction_right) {

      if (rf_tick_count == encoder_maximum) {
        rf_tick_count = encoder_minimum;
      } else {
        rf_tick_count--;
      }
    } else {
      if (rf_tick_count == encoder_minimum) {
        rf_tick_count = encoder_maximum;
      } else {
        rf_tick_count++;
      }
    }

  }

  // Increment the number of ticks
  void lf_tick() {

    // Read the value for the encoder for the right wheel
    int val = digitalRead(lf_en_b);

    if (val == LOW) {
      Direction_right = false;  // Reverse
    } else {
      Direction_right = true;  // Forward
    }

    if (Direction_right) {

      if (lf_tick_count == encoder_maximum) {
        lf_tick_count = encoder_minimum;
      } else {
        lf_tick_count++;
      }
    } else {
      if (lf_tick_count == encoder_minimum) {
        lf_tick_count = encoder_maximum;
      } else {
        lf_tick_count--;
      }
    }

  }
  void publishMsg() {
    msg_l.data = lf_tick_count;
    msg_r.data = rf_tick_count;
    RCSOFTCHECK(rcl_publish(&publisher1, &msg_l, NULL));
    RCSOFTCHECK(rcl_publish(&publisher2, &msg_r, NULL));
  }






  void loop() {
    switch (state) {
      case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        ticks_reset_flag = true;
        Serial2.println("Agent: Waiting Connection...");
        break;
      case AGENT_AVAILABLE:
        state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT) {
          destroy_entities();
        };
        Serial2.println("Agent: Avialable Connecting...");
        break;
      case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED) {
          rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        Serial2.println("Agent: Connected !");
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        state = WAITING_AGENT;
        ticks_reset_flag = true;
        Serial2.println("Agent: Disconnected !");
        break;
      default:
        break;
    }
    if (state == AGENT_CONNECTED && ticks_reset_flag) {

      ticks_reset_flag = false;
    }
    publishMsg();
//    delay(1);
  }
