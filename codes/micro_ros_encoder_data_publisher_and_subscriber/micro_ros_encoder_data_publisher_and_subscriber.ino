#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>  
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3.h>

#define ENCA1 19
#define ENCB1 21
#define ENCA2 22
#define ENCB2 23
#define DIR1 25
#define PWM1 26
#define DIR2 27
#define PWM2 32
#define LED_PIN 13

const int mChannel1 = 0;
const int mChannel2=1;
const int frequency = 5000;
const int resolution = 8;
int prev;
volatile long count1 = 0;
volatile long count2 = 0;
unsigned long previousMillis = 0;
unsigned long startmillis=0;


#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3 msg;
geometry_msgs__msg__Vector3 msg1;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void move_forward() {
  digitalWrite(DIR1, LOW);
  ledcWrite(mChannel1,msg1.x);
  digitalWrite(DIR2, LOW);
  ledcWrite(mChannel2,msg1.y);
}



void IRAM_ATTR EncoderDataA1() 
{
  if(digitalRead(ENCB1) == LOW) 
  count1--;
  else
  count1++; 
}
 
void IRAM_ATTR EncoderDataA2() 
{
  if(digitalRead(ENCB2) == LOW) 
  count2--;
  else
  count2++; 
}

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
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  msg1.x=msg->x;
  msg1.y=msg->y;
  msg1.z=msg->z;  
}


void setup() {
  set_microros_wifi_transports("Galaxy A70s338E", "lbbs3163", "192.168.80.166", 8888);

  allocator = rcl_get_default_allocator();

  pinMode(ENCA1, INPUT_PULLUP); 
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP); 
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  ledcSetup(mChannel1, frequency, resolution);
  ledcAttachPin(PWM1, mChannel1);
  ledcSetup(mChannel2, frequency, resolution);
  ledcAttachPin(PWM2, mChannel2);
  attachInterrupt(digitalPinToInterrupt(ENCA1), EncoderDataA1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), EncoderDataA2, RISING);

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "encoder_data"));
    msg.x=0;
    msg.y=0;
    msg.z=0;

  // // Set QoS profile to reliable
  // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  // custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // RCCHECK(rclc_publisher_set_qos_profile(&publisher, custom_qos_profile));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "pid_output"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg1, &subscription_callback, ON_NEW_DATA));
    
  
  prev = millis();

}

void loop() {
    msg.x=count1;
    msg.y=count2;
    msg.z=0;
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    
    move_forward();
    // if (millis() - prev > 10000){
    // digitalWrite(DIR1, LOW);
    // digitalWrite(DIR2, LOW);
    // ledcWrite(mChannel1,0);
    // ledcWrite(mChannel2,0);
    // }
}
