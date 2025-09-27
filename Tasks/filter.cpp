/**
 * @file filter.cpp
 * @author lsy
 * @brief RC滤波器类实现文件
 * @version 1.0
 * @date 2025-09-27
 */
#include "filter.hpp"

/**
 * @brief 构造函数，初始化RC滤波器参数
 * @param cutoff_freq: 截止频率，单位Hz
 * @param T: 采样周期，单位秒
 */
RCFilter::RCFilter(float cutoff_freq, float T)
    : prev_output_(0.0f), cutoff_freq_(cutoff_freq), T_(T) {
    float rc = 1.0f / (2.0f * M_PI * cutoff_freq_);
    alpha_ = T_ / (rc + T_);
}

/**
 * @brief 计算滤波器输出
 * @param input: 当前输入值
 * @return 滤波器输出值
 */
float RCFilter::output(float input) {
    prev_output_ = alpha_ * input + (1.0f - alpha_) * prev_output_;
    return prev_output_;
}