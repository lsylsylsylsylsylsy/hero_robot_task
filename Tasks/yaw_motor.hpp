/**
 * @file yaw_motor.hpp
 * @author lsy
 * @brief 云台偏航电机控制类头文件
 * @version 1.0
 * @date 2025-09-27
 */
#ifndef _YAW_MOTOR_HPP_
#define _YAW_MOTOR_HPP_
#include "dm4310_drv.hpp"
#include "HW_can.hpp"
#include "math.h"

extern Joint_Motor_t yaw_motor;

float restrict_angle(float angle);

class yaw
{
private:
    float target_yaw_;
    float current_yaw_;
    float kp_pos_;
    float kd_pos_;
    float ki_pos_;
    float dt_;
    float integral_pos_;
    float last_error_pos_;
    float output_pos_;
    float kp_vel_;
    float kd_vel_;
    float ki_vel_;
    float integral_vel_;
    float last_error_vel_;
    float current_vel_;
    float target_vel_;
    float output_vel_;
public:
    yaw(float current_yaw = -0.90f, float target_yaw = 0.0f, float kp_pos = 1.0f, float kd_pos = 0.0f, float ki_pos = 0.0f, float kp_vel = 0.0f, float kd_vel = 0.0f, float ki_vel = 0.0f);
    void update(uint16_t mode);
    void setTargetYaw(float target_yaw);
    void setTargetVel(float target_vel);
    void reset();
};
#endif