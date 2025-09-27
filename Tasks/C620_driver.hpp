/**
 * @file C620_driver.hpp
 * @author lsy
 * @brief 电机控制类头文件
 * @version 1.0
 * @date 2025-09-27
 */
#ifndef _C620_DRIVER_HPP_
#define _C620_DRIVER_HPP_
#include "HW_can.hpp"

class MotorController
{
    private:
       float kp_, ki_, kd_;
       float integral_, prev_error_;
       int16_t output_;
       float current_speed_;
       float target_speed_;
       float dt_;
       int8_t dir_;
       uint8_t id_;
    public:
    MotorController(float kp, float ki, float kd, uint8_t id, int8_t dir);
    void setGains(float kp, float ki, float kd);
    void setTargetSpeed(float speed);
    void getCurrentSpeed(float speed);
    void update();
};

#endif