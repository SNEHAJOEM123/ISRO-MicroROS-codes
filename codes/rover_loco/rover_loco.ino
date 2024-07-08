#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <math.h>

#include <sensor_msgs/msg/joy.h>


rcl_subscription_t bot_vel;
sensor_msgs__msg__Joy msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define R1_Dir  22
#define R1_Pwm  23
#define R2_Dir  19
#define R2_Pwm  21
#define R3_Dir  5
#define R3_Pwm  18
#define L1_Dir  12
#define L1_Pwm  13
#define L2_Dir  26
#define L2_Pwm  25
#define L3_Dir  27
#define L3_Pwm  14

// #define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void locomotion_callback(const void * msgin)
{  
  const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;
  // digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}



//Function to give direction and speed to the servos
void loco(float x,float y){
 
int pwm_r=abs(x);
int pwm_l=abs(y);
int dir_r=(x>0)?1:0;
int dir_l=(y>0)?1:0;

digitalWrite(R1_Dir,dir_r);
analogWrite(R1_Pwm,pwm_r);
digitalWrite(R2_Dir,dir_r);
analogWrite(R2_Pwm,pwm_r);
digitalWrite(R3_Dir,dir_r);
analogWrite(R3_Pwm,pwm_r);
digitalWrite(L1_Dir, dir_l);
analogWrite(L1_Pwm,pwm_l);
digitalWrite(L2_Dir,dir_l);
analogWrite(L2_Pwm,pwm_l);
digitalWrite(L3_Dir,dir_l);
analogWrite(L3_Pwm,pwm_l);
 

}

void setup() {
  Serial.begin(9600);
  set_microros_wifi_transports("Pixel_4459", "12345678", "192.168.122.245", 8888);
  pinMode(R1_Dir,OUTPUT);
  pinMode(R1_Pwm,OUTPUT);
  pinMode(R2_Dir,OUTPUT);
  pinMode(R2_Pwm,OUTPUT);
  pinMode(R3_Dir,OUTPUT);
  pinMode(R3_Pwm,OUTPUT);
  pinMode(L1_Dir,OUTPUT);
  pinMode(L1_Pwm,OUTPUT);
  pinMode(L2_Dir,OUTPUT);
  pinMode(L2_Pwm,OUTPUT);
  pinMode(L3_Dir,OUTPUT);
  pinMode(L3_Pwm,OUTPUT);

  // pinMode(LED_PIN, OUTPUT);
  // digitalWrite(LED_PIN, HIGH);  
 
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot1", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &bot_vel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
    "/joy"));

 
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &bot_vel, &msg, &locomotion_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  loco(float(msg.axes[0]),float(msg.axes[0]));

}