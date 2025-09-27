/**
 * @file filter.hpp
 * @author lsy
 * @brief RC滤波器类头文件
 * @version 1.0
 * @date 2025-09-27
 */
#ifndef _FILTER_HPP_
#define _FILTER_HPP_

#include "math.h"

class RCFilter
{
private:
    float prev_output_;
    float alpha_;
    float T_;
    float cutoff_freq_;
public:
    RCFilter(float cutoff_freq, float T);
    float getprev_output() const { return prev_output_; }
    float output(float input);
};

#endif