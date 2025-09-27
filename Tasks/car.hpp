/**
 * @file car.hpp
 * @author lsy
 * @brief 小车底盘控制类头文件
 * @version 1.0
 * @date 2025-09-27
 */
#ifndef _CAR_HPP_
#define _CAR_HPP_

#define _X_WIDTH 0.4f
#define _Y_WIDTH 0.396f
#define _CIRCUMFERENCE 0.308f * 3.14159f

class car
{
private:
    float x_vel_;
    float y_vel_;
    float w_;
    float motor1_v_;
    float motor2_v_;
    float motor3_v_;
    float motor4_v_;
    float angle_prev_err_;
    float angle_err_integral_;
    float kp_;
    float ki_;
    float kd_;
    float dt_ = 0.001f;
public:
    car(float x_vel, float y_vel, float w, float kp, float ki, float kd);
    void setVel(float x_vel, float y_vel, float w);
    void setAngle(float x_vel, float y_vel);
    void update();
};

#endif