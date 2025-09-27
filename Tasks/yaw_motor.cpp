/**
 * @file yaw_motor.cpp
 * @author lsy
 * @brief 云台偏航电机控制类实现文件
 * @version 1.0
 * @date 2025-09-27
 */
#include "yaw_motor.hpp"
#include "filter.hpp"
#include "system_user.hpp"

// 云台滤波器对象
extern RCFilter yaw_pos;
extern RCFilter yaw_vel;
// IMU数据对象
extern ImuDatas_t imu_datas; 
extern ImuDatas_t imu_datas_H7;

/**
 * @brief 限制角度在[-π, π]范围内
 * @param angle: 输入角度，单位弧度
 * @return 限制后的角度，单位弧度
 */
float restrict_angle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * @brief 构造函数，初始化yaw控制参数
 * @param target_yaw: 目标偏航角，单位弧度
 * @param current_yaw: 当前偏航角，单位弧度
 * @param kp_pos: 位置环比例增益
 * @param kd_pos: 位置环微分增益
 * @param ki_pos: 位置环积分增益
 * @param kp_vel: 速度环比例增益
 * @param kd_vel: 速度环微分增益
 * @param ki_vel: 速度环积分增益
 */
yaw::yaw(float target_yaw, float current_yaw, float kp_pos, float kd_pos, float ki_pos, float kp_vel, float kd_vel, float ki_vel)
    : target_yaw_(target_yaw), current_yaw_(current_yaw), kp_pos_(kp_pos), kd_pos_(kd_pos), ki_pos_(ki_pos),
      kp_vel_(kp_vel), kd_vel_(kd_vel), ki_vel_(ki_vel),
      dt_(0.001f), integral_pos_(0.0f), last_error_pos_(0.0f),
      integral_vel_(0.0f), last_error_vel_(0.0f), current_vel_(0.0f), target_vel_(0.0f), output_pos_(0.0f), output_vel_(0.0f) {}

/**
 * @brief 更新yaw控制，计算位置环和速度环的控制输出并发送给电机
 * @param mode: 控制模式，POS_MODE为位置模式，SPEED_MODE为速度模式
 */
void yaw::update(uint16_t mode) {
    // 获取当前偏航角
    current_yaw_ = restrict_angle(imu_datas_H7.euler_vals[YAW]);
    float error_pos = restrict_angle(target_yaw_ - current_yaw_);
    if (fabs(error_pos) < 0.001f) {
        error_pos = 0.0f;
    }
    // 位置环PID计算
    integral_pos_ += error_pos * dt_;
    if (integral_pos_ > 0.5f) {
        integral_pos_ = 0.5f;
    } else if (integral_pos_ < -0.5f) {
        integral_pos_ = -0.5f;
    }
    float derivative_pos = (error_pos - last_error_pos_) / dt_;
    output_pos_ = kp_pos_ * error_pos + ki_pos_ * integral_pos_ + kd_pos_ * derivative_pos;

    if (output_pos_ > 6.0f) {
        output_pos_ = 6.0f;
    } else if (output_pos_ < -6.0f) {
        output_pos_ = -6.0f;
    }
    // 获取当前偏航角速度
    current_vel_ = imu_datas_H7.gyro_vals[YAW];
    float error_vel;
    // 模式切换
    if (mode == POS_MODE) {
        error_vel = output_pos_ - current_vel_;
    } else if (mode == SPEED_MODE) {
        error_vel = target_vel_ - current_vel_;
        output_pos_ = 0.0f;
    }
    if (fabs(error_vel) < 0.001f) {
        error_vel = 0.0f;
    }
    // 速度环PID计算
    integral_vel_ += error_vel * dt_;
    if (integral_vel_ > 0.5f) {
        integral_vel_ = 0.5f;
    } else if (integral_vel_ < -0.5f) {
        integral_vel_ = -0.5f;
    }
    float derivative_vel = (error_vel - last_error_vel_) / dt_;
    output_vel_ = kp_vel_ * error_vel + ki_vel_ * integral_vel_ + kd_vel_ * derivative_vel;
    // 限制输出力矩
    if (output_vel_ > 5.0f) {
        output_vel_ = 5.0f;
    } else if (output_vel_ < -5.0f) {
        output_vel_ = -5.0f;
    }
    // 发送控制命令
    mit_ctrl(&hcan1, 0x003, 0.0f, 0.0f, 0.0f, 0.0f, output_vel_);

    last_error_pos_ = error_pos;
    last_error_vel_ = error_vel;
}

/**
 * @brief 设置目标偏航角
 * @param target_yaw: 目标偏航角，单位弧度
 */
void yaw::setTargetYaw(float target_yaw) {
  target_yaw_ = target_yaw;
}

/**
 * @brief 设置目标偏航角速度
 * @param target_vel: 目标偏航角速度，单位弧度每秒
 */
void yaw::setTargetVel(float target_vel) {
  target_vel_ = target_vel;
}

/**
 * @brief 重置位置环和速度环的积分和误差
 */
void yaw::reset() {
    integral_pos_ = 0.0f;
    last_error_pos_ = 0.0f;
    integral_vel_ = 0.0f;
    last_error_vel_ = 0.0f;
    output_pos_ = 0.0f;
    output_vel_ = 0.0f;
}