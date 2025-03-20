#include "m5ros_const.h"
#include "utils.h"

// micro-ROS settings
rcl_subscription_t subscriber;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Cybergear settings
// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t master_can_id = MASTER_CAN_ID;
std::vector<uint8_t> motor_ids = MOTOR_IDS;
std::vector<float> speed_command(motor_ids.size(), 0.0f);
std::vector<float> servo_command(SERVO_COMMAND_NUM, 0.0f);
#ifdef USE_ESP32_CAN
CybergearCanInterfaceEsp32 interface;
#else
CybergearCanInterfaceMcp CAN0;
#endif
CybergearController controller = CybergearController(master_can_id);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_ADDR);

void subscription_callback(const void * msgin){
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  // データをvectorに変換
  std::vector<float> data_vector(msg->data.data, msg->data.data + msg->data.size);
  // 各要素を表示
  for(size_t i = 0; i < data_vector.size(); i++) {
    M5.Lcd.printf("Data[%d]: %.2f ", i, data_vector[i]);
    if(i < motor_ids.size()) {
      speed_command[i] = data_vector[i];
    } else {
      servo_command[i - motor_ids.size()] = data_vector[i];
    }
  }
  // CyberGearController
  controller.send_speed_command(motor_ids, speed_command);
  // Servo controller
  pwm.setPWM(SERVO_CHANNEL, 0, angleToPulse(servo_command[0], SERVOMIN, SERVOMAX));
  pwm.setPWM(ROLLER_CNANNEL, 0, servo_command[1]);
  if (servo_command[4] >= 1.0) {
    Serial.begin(115200);
    Wire.begin();
    CAN0.init();
    M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setCursor(0,0,2);
    M5.Lcd.printf("Reset Motors.\n");
    controller.init(motor_ids, MODE_SPEED, &CAN0);
    controller.enable_motors();
    M5.Lcd.printf("Reset CyberGear.\n");
  } else {
    ;
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
  while (!wait_for_agent(TIMEOUT_MS, INTERVAL_MS)) {
    Serial.println("No micro_ros_agent found. Stopping setup.");
  }
  M5.Lcd.print("initialize micro-ros\n");

  //initialize cybergear
  Serial.begin(115200);
  Wire.begin();
  CAN0.init();
  M5.Lcd.printf("initialize can\n");

  controller.init(motor_ids, MODE_SPEED, &CAN0);
  M5.Lcd.printf("initialize controller\n");

  controller.enable_motors();
  M5.Lcd.printf("initialize motor\n");

  //initialize Servo2 module
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  int pulse0 = map(0, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(SERVO_CHANNEL, 0, pulse0);
  delay(1000);

  // for roller
  pwm.setPWM(ROLLER_CNANNEL, 0, 1060);  //指定の速度で射出モーターを回転
  delay(1000);
  M5.Lcd.printf("initialize servo motor\n");

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "micro_ros_arduino_subscriber"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  M5.Lcd.printf("initialize node\n");

  // メッセージの初期化
  msg.data.capacity = 10;
  msg.data.size = 0;
  msg.data.data = (float*)malloc(msg.data.capacity * sizeof(float));
  M5.Lcd.print("all initialize");
}

void loop() {
  //update m5 status
  M5.update();
  M5.Lcd.print("Cybergear Controller\n");
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  M5.Lcd.clear(TFT_BLACK);
  M5.Lcd.setCursor(0,0,2);
}