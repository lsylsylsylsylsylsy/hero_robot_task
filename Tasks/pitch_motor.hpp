/**
 *******************************************************************************
 * @file      :pitch_motor.hpp
 * @brief     :  俯仰电机控制头文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-06-15      lsy             首次发布
 *******************************************************************************
 * @attention :
 *******************************************************************************
 */
#ifndef _PITCH_MOTOR_HPP_
#define _PITCH_MOTOR_HPP_
#include "dm4310_drv.hpp"
#include "HW_fdcan.hpp"
#include "math.h"

extern Joint_Motor_t pitch_motor;

class pitch
{
private:
    float target_pitch_;
    float current_pitch_;
    float kp_pos_;
    float kd_pos_;
    float ki_pos_;
    float dt_;
    float integral_pos_;
    float last_error_pos_;
    float kp_vel_;
    float kd_vel_;
    float ki_vel_;
    float integral_vel_;
    float last_error_vel_;
    float current_vel_;
    float target_vel_;
    float output_vel_;
public:
    pitch(float current_pitch = -0.90f, float target_pitch = 0.0f, float kp_pos = 1.0f, float kd_pos = 0.0f, float ki_pos = 0.0f, float kp_vel = 0.0f, float kd_vel = 0.0f, float ki_vel = 0.0f);
    void update(uint16_t mode);
    void setTargetPitch(float target_pitch);
    void setTargetVel(float target_vel);
};
#endif