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

#include "Adafruit_PWMServoDriver.h"

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
std::vector<uint8_t> motor_ids = {127, 126, 125, 124};
std::vector<float> speeds = {0.0f, 0.0f, 0.0f, 0.0f};
std::vector<float> speed_command;
std::vector<float> servo_command;
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp CAN0;
#endif
CybergearController controller = CybergearController(MASTER_CAN_ID);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){check_connect();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// micro_ros_agentの確認用のタイムアウト設定
#define AGENT_CHECK_TIMEOUT_MS 10000
#define AGENT_CHECK_INTERVAL_MS 500

// Servo motor settings
#define SERVO_ADDR 0x40
#define SERVOMIN  150  // サーボパルス幅の最小値（マイクロ秒）
#define SERVOMAX  600  // サーボパルス幅の最大値（マイクロ秒）
#define USMIN     500  // サーボの最小範囲（マイクロ秒）
#define USMAX     2500 // サーボの最大範囲（マイクロ秒）
#define SERVO_FREQ 50  // サーボのPWM周波数（Hz）
#define SERVO_NUM 4 // サーボの数

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_ADDR);

// 角度をパルス幅に変換する関数
uint16_t angleToPulse(float angle) {
  // 角度（0〜180）をパルス幅（SERVOMIN〜SERVOMAX）に変換
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

bool wait_for_agent() {
  unsigned long start_time = millis();
  
  while (millis() - start_time < AGENT_CHECK_TIMEOUT_MS) {
    // micro_ros_agentとの接続確認
    bool agent_available = false;
    
    // pingの送信を試みる
    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
      M5.Lcd.printf("micro_ros_agent found!");
      return true;
    }
    
    M5.Lcd.printf("Waiting for micro_ros_agent...");
    delay(AGENT_CHECK_INTERVAL_MS);
  }
  
  M5.Lcd.printf("micro_ros_agent not found within timeout period");
  return false;
}


void check_connect(){
  // update m5 status
  M5.Lcd.print("connecting...");
}

void subscription_callback(const void * msgin){
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // データをvectorに変換
  std::vector<float> data_vector(msg->data.data, msg->data.data + msg->data.size);
  // 各要素を表示
  for(size_t i = 0; i < data_vector.size(); i++) {
    M5.Lcd.printf("Data[%d]: %.2f\n", i, data_vector[i]);
    if(i < motor_ids.size()) {
      speed_command[i] = data_vector[i];
    } else {
      servo_command[i - motor_ids.size()] = data_vector[i];
    }
  }
  // CyberGearController
  controller.send_speed_command(motor_ids, speed_command);
  // Servo controller
  for (int i = 0; i < SERVO_NUM; i++) {
    pwm.setPWM(i, 0, angleToPulse(servo_command[i]));
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
  // micro_ros_agentの存在確認
  while (!wait_for_agent()) {
    Serial.println("No micro_ros_agent found. Stopping setup.");
  }
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

  //initialize Servo2 module
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

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