#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>  
#include <rclc/executor.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>

//R1
#define DIR1 4 
#define PWM1 5 

//R2
#define DIR2 14
#define PWM2 15 

//R3
#define DIR3 18
#define PWM3 19 

//L1
#define DIR4 22 
#define PWM4 23

//L2
#define DIR5 25 
#define PWM5 26 

//L3
#define DIR6 27
#define PWM6 32
#define LED_PIN 13

const int mChannel1 = 0;
const int mChannel2= 1;
const int mChannel3 = 2;
const int mChannel4 = 3;
const int mChannel5 = 4;
const int mChannel6 = 5;
const int frequency = 5000;
const int resolution = 8;


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
geometry_msgs__msg__Twist msg1;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void subscription_callback(const void * msgin)
{  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  msg1.linear.x=msg->linear.x;
  msg1.linear.y=msg->linear.y;
  msg1.linear.z=msg->linear.z;
  msg1.angular.x=msg->angular.x;
  msg1.angular.y=msg->angular.y;
  msg1.angular.z=msg->angular.z;  
}


void setup() {
  set_microros_wifi_transports("sjm-Inspiron-3584", "12345678", "10.42.0.1", 8888);

  allocator = rcl_get_default_allocator();

  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);
  pinMode(PWM5, OUTPUT);
  pinMode(DIR5, OUTPUT);
  pinMode(PWM6, OUTPUT);
  pinMode(DIR6, OUTPUT);
  ledcSetup(mChannel1, frequency, resolution);
  ledcAttachPin(PWM1, mChannel1);
  ledcSetup(mChannel2, frequency, resolution);
  ledcAttachPin(PWM2, mChannel2);
  ledcSetup(mChannel3, frequency, resolution);
  ledcAttachPin(PWM3, mChannel3);
  ledcSetup(mChannel4, frequency, resolution);
  ledcAttachPin(PWM4, mChannel4);
  ledcSetup(mChannel5, frequency, resolution);
  ledcAttachPin(PWM5, mChannel5);
  ledcSetup(mChannel6, frequency, resolution);
  ledcAttachPin(PWM6, mChannel6);
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // // Set QoS profile to reliable
  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // RCCHECK(rclc_publisher_set_qos_profile(&publisher, custom_qos_profile));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "pwm_output"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg1, &subscription_callback, ON_NEW_DATA));
  

}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    if(msg1.linear.x>0 && msg1.linear.y>0 && msg1.linear.z>0 && msg1.angular.x>0 && msg1.angular.y>0 && msg1.angular.z>0){
      digitalWrite(DIR1, LOW);
      ledcWrite(mChannel1,msg1.linear.x);
      digitalWrite(DIR2, LOW);
      ledcWrite(mChannel2,msg1.linear.y);
      digitalWrite(DIR3, LOW);
      ledcWrite(mChannel3,msg1.linear.z);
      digitalWrite(DIR4, LOW);
      ledcWrite(mChannel4,msg1.angular.x);
      digitalWrite(DIR5, LOW);
      ledcWrite(mChannel5,msg1.angular.y);
      digitalWrite(DIR6, LOW);
      ledcWrite(mChannel6,msg1.angular.z);
    }

    else if(msg1.linear.x<0 && msg1.linear.y<0 && msg1.linear.z<0 && msg1.angular.x<0 && msg1.angular.y<0 && msg1.angular.z<0)
    {
      digitalWrite(DIR1, HIGH);
      ledcWrite(mChannel1,-msg1.linear.x);
      digitalWrite(DIR2, HIGH);
      ledcWrite(mChannel2,-msg1.linear.y);
      digitalWrite(DIR3, HIGH);
      ledcWrite(mChannel3,-msg1.linear.z);
      digitalWrite(DIR4, HIGH);
      ledcWrite(mChannel4,-msg1.angular.x);
      digitalWrite(DIR5, HIGH);
      ledcWrite(mChannel5,-msg1.angular.y);
      digitalWrite(DIR6, HIGH);
      ledcWrite(mChannel6,-msg1.angular.z);
    }

    else if(msg1.linear.x==0 && msg1.linear.y==0 && msg1.linear.z==0 && msg1.angular.x==0 && msg1.angular.y==0 && msg1.angular.z==0){
      digitalWrite(DIR1, LOW);
      ledcWrite(mChannel1,msg1.linear.x);
      digitalWrite(DIR2, LOW);
      ledcWrite(mChannel2,msg1.linear.y);
      digitalWrite(DIR3, LOW);
      ledcWrite(mChannel3,msg1.linear.z);
      digitalWrite(DIR4, LOW);
      ledcWrite(mChannel4,msg1.angular.x);
      digitalWrite(DIR5, LOW);
      ledcWrite(mChannel5,msg1.angular.y);
      digitalWrite(DIR6, LOW);
      ledcWrite(mChannel6,msg1.angular.z);
    }
}
