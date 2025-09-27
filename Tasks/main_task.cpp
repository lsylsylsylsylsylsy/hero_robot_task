/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :  主任务文件
* @history   :
*  Version     Date            Author          Note
*   V1.0.0   2025-09-27      lsy             首次发布
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_fdcan.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "pitch_motor.hpp"
#include "filter.hpp"
#include "imu.hpp"
#include "spi.h"
#include "ahrs.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;
float pitch_speed = 0.0f;
float pitch_angle = 0.0f;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen] __attribute__((section(".RAM_D1")));
remote_control::DT7 *rc_ptr;
Joint_Motor_t pitch_motor;
RCFilter pitch_pos(50, 0.001f);
RCFilter pitch_vel(5, 0.001f);
pitch PitchMotor(0.0f, 0.0f, 7.0f, 0.0f, 0.0f, 3.0f, 0.0f, 1.0f);
uint8_t data_to_C[8] = {0};
bool stop_flag = false;

hello_world::imu::BMI088 Imu;
hello_world::ahrs::Mahony mahony_filter(1000.0f, 0.5f, 0.0f);
uint32_t count = 0;

ImuDatas_t imu_datas;

void Send_To_C(){
  FDCAN_Send_Msg(&hfdcan1, data_to_C, 0x004, 8);
  uint16_t u_temp = float_to_uint(imu_datas.euler_vals[YAW], -M_PI, M_PI, 16);
  data_to_C[0] = u_temp >> 8;
  data_to_C[1] = u_temp & 0xFF;
  u_temp = float_to_uint(imu_datas.gyro_vals[YAW], -20.0f, 20.0f, 16);
  data_to_C[2] = u_temp >> 8;
  data_to_C[3] = u_temp & 0xFF;
  FDCAN_Send_Msg(&hfdcan1, data_to_C, 0x014, 8);
  count++;
}

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  // 开启FDCAN
  FdcanFilter_Init(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan3);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
  // 初始化IMU
  hello_world::imu::BMI088HWConfig imu_hw_config;
  imu_hw_config.hspi = &hspi2;
  imu_hw_config.acc_cs_port = GPIOC;
  imu_hw_config.acc_cs_pin = GPIO_PIN_0;
  imu_hw_config.gyro_cs_port = GPIOC;
  imu_hw_config.gyro_cs_pin = GPIO_PIN_3;
  float rot_mat_flatten[9] = {
      1.0f, 0.0f, 0.0f,
      0.0f, 1.0f, 0.0f,
      0.0f, 0.0f, 1.0f};
  hello_world::imu::BMI088Config imu_config;
  imu_config.acc_range = hello_world::imu::kBMI088AccRange12G;
  imu_config.acc_odr = hello_world::imu::kBMI088AccOdr1600;
  imu_config.acc_osr = hello_world::imu::kBMI088AccOsr4;
  imu_config.gyro_range = hello_world::imu::kBMI088GyroRange1000Dps;
  imu_config.gyro_odr_fbw = hello_world::imu::kBMI088GyroOdrFbw1000_116;
  Imu.init(imu_hw_config, rot_mat_flatten, imu_config);
  if(Imu.imuInit(true) != hello_world::imu::kBMI088ErrStateNoErr){
    //Error_Handler();
  }
  // 开启pitch电机
  joint_motor_init(&pitch_motor, 0x009, MIT_MODE);
  disable_motor_mode(&hfdcan3, 0x009, MIT_MODE);
  HAL_Delay(1);
}

void MainTask(void) {
  tick++; 
  if (tick < 1000) {
    enable_motor_mode(&hfdcan3, 0x009, pitch_motor.mode);
    mit_ctrl(&hfdcan3, 0x009, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  else{
    pitch_angle += pitch_speed * 0.001f;
    if(pitch_angle > 0.80f) pitch_angle = 0.80f;
    if(pitch_angle < -0.27f) pitch_angle = -0.27f;
    Imu.getData(imu_datas.acc_vals, imu_datas.gyro_vals, nullptr);
    mahony_filter.update(imu_datas.acc_vals, imu_datas.gyro_vals);
    mahony_filter.getEulerAngle(imu_datas.euler_vals);
    if (stop_flag == false) {
      PitchMotor.setTargetPitch(pitch_angle);
      PitchMotor.update(POS_MODE);
    }
    else {
      mit_ctrl(&hfdcan3, 0x009, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    Send_To_C();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart5) {
    if (Size == remote_control::kRcRxDataLen) {
      // TODO:在这里进行看门狗刷新
      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);
  }
}
