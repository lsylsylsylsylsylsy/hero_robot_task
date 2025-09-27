/**
 *******************************************************************************
 * @file      :pitch_motor.cpp
 * @brief     :  俯仰电机控制文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-06-15      lsy             首次发布
 *******************************************************************************
 * @attention :
 *******************************************************************************
 */
#include "pitch_motor.hpp"
#include "filter.hpp"

// 电机滤波器和IMU数据
extern RCFilter pitch_pos;
extern RCFilter pitch_vel;
extern ImuDatas_t imu_datas;

/**
 * @brief 构造函数，初始化电机控制参数
 * @param current_pitch 当前俯仰角（弧度）
 * @param target_pitch 目标俯仰角（弧度）
 * @param kp_pos 位置环比例增益
 * @param kd_pos 位置环微分增益
 * @param ki_pos 位置环积分增益
 * @param kp_vel 速度环比例增益
 * @param kd_vel 速度环微分增益
 * @param ki_vel 速度环积分增益
 * @return None
 */
pitch::pitch(float target_pitch, float current_pitch, float kp_pos, float kd_pos, float ki_pos, float kp_vel, float kd_vel, float ki_vel)
    : target_pitch_(target_pitch), current_pitch_(current_pitch), kp_pos_(kp_pos), kd_pos_(kd_pos), ki_pos_(ki_pos),
      kp_vel_(kp_vel), kd_vel_(kd_vel), ki_vel_(ki_vel),
      dt_(0.001f), integral_pos_(0.0f), last_error_pos_(0.0f),
      integral_vel_(0.0f), last_error_vel_(0.0f), current_vel_(0.0f), target_vel_(0.0f), output_vel_(0.0f) {}

/**
 * @brief 更新电机控制，计算位置和速度环输出
 * @param mode 控制模式，POS_MODE位置环，SPEED_MODE速度环
 * @return None
 */
void pitch::update(uint16_t mode) {
    // 获取当前俯仰角
    current_pitch_ = imu_datas.euler_vals[ROLL];
    float error_pos = target_pitch_ - current_pitch_;
    if (error_pos > M_PI)
        error_pos -= 2 * M_PI;
    else if (error_pos < -M_PI)
        error_pos += 2 * M_PI;
    if (fabs(error_pos) < 0.01f) {
        error_pos = 0.0f;
    }
    // 位置环PID计算
    integral_pos_ += error_pos * dt_;
    float derivative_pos = (error_pos - last_error_pos_) / dt_;
    float output_pos = kp_pos_ * error_pos + ki_pos_ * integral_pos_ + kd_pos_ * derivative_pos;
    // 获取当前速度
    current_vel_ = imu_datas.gyro_vals[ROLL];
    float error_vel;
    // 模式选择
    if (mode == POS_MODE) {
        error_vel = output_pos - current_vel_;
    } else if (mode == SPEED_MODE) {
        error_vel = target_vel_ - current_vel_;
    }
    if (fabs(error_vel) < 0.01f) {
        error_vel = 0.0f;
    }
    // 速度环PID计算
    integral_vel_ += error_vel * dt_;
    float derivative_vel = (error_vel - last_error_vel_) / dt_;
    output_vel_ = kp_vel_ * error_vel + ki_vel_ * integral_vel_ + kd_vel_ * derivative_vel;
    // 速度环限幅
    if (output_vel_ > 15.0f) {
        output_vel_ = 15.0f;
    } else if (output_vel_ < -15.0f) {
        output_vel_ = -15.0f;
    }
    // 位置环饱和时速度环不工作
    if ((pitch_pos.getprev_output() > 0.80f && output_vel_ > 0.0f) || (pitch_pos.getprev_output() < -0.20f && output_vel_ < 0.0f)) {
        output_vel_ = 0.0f;
    }
    // 发送速度指令给电机，前面加一个重力补偿
    mit_ctrl(&hfdcan3, 0x009, 0.0f, 0.0f, 0.0f, 0.0f, output_vel_ + 1.0f * cos(imu_datas.euler_vals[ROLL]));
    // 保存上次误差
    last_error_pos_ = error_pos;
    last_error_vel_ = error_vel;
}

/**
 * @brief 设置目标俯仰角
 * @param target_pitch 目标俯仰角（弧度）
 * @return None
 */
void pitch::setTargetPitch(float target_pitch) {
  target_pitch_ = target_pitch;
}

/**
 * @brief 设置目标速度
 * @param target_vel 目标速度（弧度/秒）
 * @return None
 */
void pitch::setTargetVel(float target_vel) {
  target_vel_ = target_vel;
}