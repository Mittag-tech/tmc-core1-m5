#ifndef UTIL_H
#define UTIL_H

#include <M5Stack.h>
#include <mcp_can.h>
#include <micro_ros_arduino.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "Adafruit_PWMServoDriver.h"

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include "cybergear_controller.hh"
#include "cybergear_can_interface_esp32.hh"
#include "cybergear_can_interface_mcp.hh"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){check_connect();}}

// 角度をパルス幅に変換する関数
uint16_t angleToPulse(float angle, int servomin, int servomax) {
    // 角度（0〜180）をパルス幅（SERVOMIN〜SERVOMAX）に変換
    uint16_t pulse = map(angle, 0, 180, servomin, servomax);
    return pulse;
  }
  
bool wait_for_agent(int timeout, int interval) {
    unsigned long start_time = millis();

    while (millis() - start_time < timeout) {
        // micro_ros_agentとの接続確認
        bool agent_available = false;
        
        // pingの送信を試みる
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        M5.Lcd.printf("micro_ros_agent found!");
        return true;
        }
        
        M5.Lcd.printf("Waiting for micro_ros_agent...");
        M5.Lcd.setCursor(10,0,2);
        delay(interval);
    }

    M5.Lcd.printf("micro_ros_agent not found within timeout period");
    return false;
}
  
void check_connect(){
    // update m5 status
    M5.Lcd.print("connecting...");
    M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setCursor(0,0,2);
}

#endif