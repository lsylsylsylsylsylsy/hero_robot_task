/**
 * @file C620_driver.cpp
 * @author lsy
 * @brief 电机控制类实现文件
 * @version 1.0
 * @date 2025-09-27
 */
#include "C620_driver.hpp"

// 电机控制数据
uint8_t data[8]={0,0,0,0,0,0,0,0};

/**
 * @brief 构造函数，初始化PID参数和电机ID
 * @param kp: 比例增益
 * @param ki: 积分增益
 * @param kd: 微分增益
 * @param id: 电机ID（1-4）
 * @param dir: 电机方向（1或-1）
 */
MotorController::MotorController(float kp, float ki, float kd, uint8_t id, int8_t dir)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), prev_error_(0.0f),
      output_(0), current_speed_(0.0f), target_speed_(0.0f), id_(id), dt_(0.001f), dir_(dir) {}

/**
 * @brief 设置PID参数
 * @param kp: 比例增益
 * @param ki: 积分增益
 * @param kd: 微分增益
 */
void MotorController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd; 
}

/**
 * @brief 设置目标速度
 * @param speed: 目标速度，单位为RPM
 */
void MotorController::setTargetSpeed(float speed) { 
    target_speed_ = speed * dir_; 
}

/**
 * @brief 更新当前速度
 * @param speed: 当前速度，单位为RPM
 */
void MotorController::getCurrentSpeed(float speed) { 
    current_speed_ = speed; 
}

/**
 * @brief 更新电机控制，计算新的控制输出并发送CAN消息
 */
void MotorController::update() {
  // 计算PID控制输出
  float error = target_speed_ - current_speed_;
  integral_ += error * dt_;
  float derivative = (error - prev_error_) / dt_;

  output_ = static_cast<int16_t>(kp_ * error + ki_ * integral_ + kd_ * derivative);
  prev_error_ = error;

  // 限制输出范围
  if (output_ > 16384) output_ = 16384;
  if (output_ < -16384) output_ = -16384;

  // 发送CAN消息
  switch (id_)
  {
    case 1:
        data[0] = static_cast<uint8_t>(output_ >> 8);
        data[1] = static_cast<uint8_t>(output_ & 0xFF);
        break;
    case 2:
        data[2] = static_cast<uint8_t>(output_ >> 8);
        data[3] = static_cast<uint8_t>(output_ & 0xFF);
        break;
    case 3:
        data[4] = static_cast<uint8_t>(output_ >> 8);
        data[5] = static_cast<uint8_t>(output_ & 0xFF);
        break;
    case 4:
        data[6] = static_cast<uint8_t>(output_ >> 8);
        data[7] = static_cast<uint8_t>(output_ & 0xFF);
        break;
  };
}