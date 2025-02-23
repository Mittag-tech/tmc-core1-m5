//
// micro-ROS subscriber program for CyberGear Controller by M5Stack
//
#include <M5Stack.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include <mcp_can.h>
#include "cybergear_controller.hh"
#include "cybergear_can_interface_esp32.hh"
#include "cybergear_can_interface_mcp.hh"

// micro-ROS settings
rcl_subscription_t subscriber;
// std_msgs__msg__Int32 msg;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Cybergear settings
// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t MASTER_CAN_ID = 0x00;
std::vector<uint8_t> motor_ids = {0x7F, 0x7E};
std::vector<float> speeds = {0.0f, 0.0f};
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp CAN0;
#endif
CybergearController controller = CybergearController(MASTER_CAN_ID);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){check_connect();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void check_connect(){
  // update m5 status
  M5.Lcd.print("connecting...");
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // データをvectorに変換
  std::vector<float> data_vector(msg->data.data, msg->data.data + msg->data.size);
  // CyberGearController
  controller.send_speed_command(motor_ids, data_vector);
  // 各要素を表示
  for(size_t i = 0; i < data_vector.size(); i++) {
    M5.Lcd.printf("Data[%d]: %.2f\n", i, data_vector[i]);
  }
}

void setup() {
  // initialize M5
  M5.begin();
  M5.Power.begin();
  M5.Lcd.setTextSize(2); 
  M5.Lcd.clear(BLACK);
  M5.Lcd.setCursor(0,0,2);
  M5.Lcd.print("initialize start\n");
  
  // initialize micro-ros
  set_microros_transports();
  M5.Lcd.print("initialize micro-ros\n");

  //initialize cybergear
  Serial.begin(115200);
  CAN0.init();
  M5.Lcd.printf("initialize can\n");
  controller.init(motor_ids, MODE_SPEED, &CAN0);
  M5.Lcd.printf("initialize controller\n");
  controller.enable_motors();
  controller.send_position_command(motor_ids, speeds);
  M5.Lcd.printf("initialize motor\n");

  M5.Lcd.print("all initialize");
  delay(1000);
  M5.Lcd.clear(TFT_BLACK);
  M5.Lcd.setCursor(0,0,2);

  delay(2000);
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // メッセージの初期化
  msg.data.capacity = 10;
  msg.data.size = 0;
  msg.data.data = (float*)malloc(msg.data.capacity * sizeof(float));
}

void loop() {
  //update m5 status
  M5.Lcd.print("Cybergear Controller\n");
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(1000);
  M5.Lcd.clear(TFT_BLACK);
  M5.Lcd.setCursor(0,0,2);
}