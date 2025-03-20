#ifndef M5ROS_CONST_H
#define M5ROS_CONST_H

// cybergear settings
#define MASTER_CAN_ID 0x00
#define MOTOR_IDS {127, 125, 126, 124}

// micro_ros_agentの確認用のタイムアウト設定
#define AGENT_CHECK_TIMEOUT_MS 10000
#define AGENT_CHECK_INTERVAL_MS 500

// Servo motor settings
#define SERVO_COMMAND_NUM 5
#define SERVO_ADDR 0x40
#define SERVOMIN  150  // サーボパルス幅の最小値（マイクロ秒）
#define SERVOMAX  600  // サーボパルス幅の最大値（マイクロ秒）
#define USMIN     500  // サーボの最小範囲（マイクロ秒）
#define USMAX     2500 // サーボの最大範囲（マイクロ秒）
#define SERVO_FREQ 50  // サーボのPWM周波数（Hz）
#define SERVO_NUM 4 // サーボの数
#define SERVO_CHANNEL 3 

// roller settings
#define ROLLER_CNANNEL 1

#endif