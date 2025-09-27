/**
 * @file car.cpp
 * @author lsy
 * @brief 小车底盘控制类实现文件
 * @version 1.0
 * @date 2025-09-27
 */
#include "car.hpp"
#include "C620_driver.hpp"
#include "HW_can.hpp"
#include "system_user.hpp"
#include "math.h"
#include "dm4310_drv.hpp"
#include "filter.hpp"

extern CAN_HandleTypeDef hcan2;
// 电机控制对象
extern MotorController motor1;
extern MotorController motor2;
extern MotorController motor3;
extern MotorController motor4;
extern uint8_t data[8];

// 云台陀螺仪数据
ImuDatas_t imu_datas;
// 云台低通滤波器对象
extern RCFilter yaw_pos;
extern Joint_Motor_t yaw_motor;

/**
 * @brief 构造函数，初始化小车参数
 * @param x_vel: 线速度x，单位m/s
 * @param y_vel: 线速度y，单位m/s
 * @param w: 角速度，单位rad/s
 * @param kp: 角度控制比例增益
 * @param ki: 角度控制积分增益
 * @param kd: 角度控制微分增益
 */
car::car(float x_vel, float y_vel, float w, float kp, float ki, float kd)
    : x_vel_(x_vel), y_vel_(y_vel), w_(w), motor1_v_(0.0f), motor2_v_(0.0f), motor3_v_(0.0f), motor4_v_(0.0f), angle_prev_err_(0.0f), angle_err_integral_(0.0f), kp_(kp), ki_(ki), kd_(kd), dt_(0.001f) {}

/**
 * @brief 速度模式控制小车底盘
 * @param x_vel: 线速度x，单位m/s
 * @param y_vel: 线速度y，单位m/s
 * @param w: 角速度，单位rad/s
 */
void car::setVel(float x_vel, float y_vel, float w) {
    x_vel_ = x_vel;
    y_vel_ = y_vel;
    w_ = w;
}

/**
 * @brief 角度模式控制小车底盘，底盘自动调整角速度以保持云台水平
 * @param x_vel: 线速度x，单位m/s
 * @param y_vel: 线速度y，单位m/s
 */
void car::setAngle(float x_vel, float y_vel) {
    // 计算期望角速度
    x_vel_ = x_vel;
    y_vel_ = y_vel;
    float angle_err = yaw_motor.para.pos + 0.90f;
    if (fabs(angle_err) < 0.03f) {
        angle_err = 0.0f;
    }
    if (angle_err > M_PI)
        angle_err -= 2 * M_PI;
    else if (angle_err < -M_PI)
        angle_err += 2 * M_PI;
    // PID控制计算角速度
    angle_err_integral_ += angle_err * dt_;
    float angle_derivative = (angle_err - angle_prev_err_) / dt_;
    w_ = kp_ * angle_err + ki_ * angle_err_integral_ + kd_ * angle_derivative;
    if (fabs(w_) < 0.1f) {
        w_ = 0.0f;
    }
    // 限制最大角速度
    if (w_ > 10.0f) {
        w_ = 10.0f;
    } else if (w_ < -10.0f) {
        w_ = -10.0f;
    }
    angle_prev_err_ = angle_err;
}


/**
 * @brief 更新小车底盘控制，计算各个电机的目标速度并发送CAN消息
 */
void car::update() {
    // 计算各个电机的线速度，单位m/s
    motor1_v_ = x_vel_ + y_vel_ - w_ * (_X_WIDTH + _Y_WIDTH) / 2;
    motor4_v_ = x_vel_ - y_vel_ + w_ * (_X_WIDTH + _Y_WIDTH) / 2;
    motor2_v_ = x_vel_ - y_vel_ - w_ * (_X_WIDTH + _Y_WIDTH) / 2;
    motor3_v_ = x_vel_ + y_vel_ + w_ * (_X_WIDTH + _Y_WIDTH) / 2;
    // 设置电机目标速度，单位RPM
    motor1.setTargetSpeed(motor1_v_ / _CIRCUMFERENCE * 60);
    motor2.setTargetSpeed(motor2_v_ / _CIRCUMFERENCE * 60);
    motor3.setTargetSpeed(motor3_v_ / _CIRCUMFERENCE * 60);
    motor4.setTargetSpeed(motor4_v_ / _CIRCUMFERENCE * 60);
    // 更新电机控制
    motor1.update();
    motor2.update();
    motor3.update();
    motor4.update();
    // 发送CAN消息
    CAN_Send_Msg(&hcan2, data, 0x200, 8);
}
