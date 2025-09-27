/**
 *******************************************************************************
 * @file      :filter.hpp
 * @brief     :  一阶RC低通滤波器头文件
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2024-06-15      lsy             首次发布
 *******************************************************************************
 * @attention :
 *******************************************************************************
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