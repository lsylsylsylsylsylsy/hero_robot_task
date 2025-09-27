/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :主任务文件
* @history   :
*  Version     Date            Author          Note
*  V1.0      2025-09-27      lsy             1. Initial version
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
#include "HW_can.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "C620_driver.hpp"
#include "car.hpp"
#include "system_user.hpp"
#include "imu.hpp"
#include "spi.h"
#include "ahrs.hpp"
#include "dm4310_drv.hpp"
#include "yaw_motor.hpp"
#include "filter.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

uint32_t tick = 0;

// 电机控制对象
MotorController motor1(3.0f, 5.0f, 0.1f, 1, 1);
MotorController motor2(3.0f, 5.0f, 0.11f, 2, 1);
MotorController motor3(3.0f, 5.0f, 0.1f, 3, -1);
MotorController motor4(3.0f, 5.0f, 0.1f, 4, -1);
// 小车底盘控制对象
car Car(0.0f, 0.0f, 0.0f, 5.0f, 0.5f, 0.1f);
// IMU对象和滤波器对象
hello_world::imu::BMI088 Imu;
hello_world::ahrs::Mahony mahony_filter(1000.0f, 0.5f, 0.0f);
// 发送给底盘的数据
extern uint8_t data[8];
// 遥控器对象
namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
remote_control::DT7 *rc_ptr;
// YAW电机对象
Joint_Motor_t yaw_motor;
yaw YawMotor(0.0f, 0.0f, 4.0f, 0.4f, 1.0f, 3.0f, 0.0f, 2.0f);
// YAW电机低通滤波器对象
RCFilter yaw_pos(50.0f, 0.001f);
RCFilter yaw_vel(5.0f, 0.001f);
// 目标偏航角和云台夹角
float target_yaw = 0.0f;
float theta = 0.0f;
// 云台陀螺仪数据
ImuDatas_t imu_datas_H7;
uint8_t data_to_H7[8]={0};

void Send_To_H7()
{
  // 发送给H7的数据
  float pitch_angle = rc_ptr->rc_rv();
  uint16_t u_temp = float_to_uint(pitch_angle, -1.0f, 1.0f, 16);
  data_to_H7[0] = u_temp >> 8;
  data_to_H7[1] = u_temp & 0xFF;
  CAN_Send_Msg(&hcan1, data_to_H7, 0x114, 8);
}

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void)
{
  RobotInit();

  // 开启CAN1和CAN2
  CanFilter_Init(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
  // 初始化IMU
  hello_world::imu::BMI088HWConfig imu_hw_config;
  imu_hw_config.hspi = &hspi1;
  imu_hw_config.acc_cs_port = GPIOA;
  imu_hw_config.acc_cs_pin = GPIO_PIN_4;
  imu_hw_config.gyro_cs_port = GPIOB;
  imu_hw_config.gyro_cs_pin = GPIO_PIN_0;
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
  Imu.imuInit(true);
  // 初始化yaw电机
  joint_motor_init(&yaw_motor, 0x003, MIT_MODE);
  //  电机控制电流置零
  uint8_t zero[8] = {0};
  CAN_Send_Msg(&hcan2, zero, 0x200, 8);
}

void MainTask(void)
{
  tick++;
  if (tick < 1000)
  {
    //等待遥控器开启
    enable_motor_mode(&hcan1, 0x003, yaw_motor.mode);
    uint8_t zero[8] = {0};
    CAN_Send_Msg(&hcan2, zero, 0x200, 8);
    mit_ctrl(&hcan1, 0x003, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }
  else if (tick >= 1000 && rc_ptr->rc_r_switch() == remote_control::kSwitchStateMid)
  {
    enable_motor_mode(&hcan1, 0x003, yaw_motor.mode);
    //更新陀螺仪数据
    Imu.getData(imu_datas.acc_vals, imu_datas.gyro_vals, nullptr);
    mahony_filter.update(imu_datas.acc_vals, imu_datas.gyro_vals);
    mahony_filter.getEulerAngle(imu_datas.euler_vals);

    theta = restrict_angle(0.90f + yaw_motor.para.pos);
    if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateUp)
    {
      //跟随模式
      target_yaw -= 0.002f * rc_ptr->rc_rh();
      target_yaw = restrict_angle(target_yaw);

      Car.setAngle(10.0f * rc_ptr->rc_lv() * cos(theta) + 10.0f * rc_ptr->rc_lh() * sin(theta), 10.0f * rc_ptr->rc_lh() * cos(theta) - 10.0f * rc_ptr->rc_lv() * sin(theta));
    }
    else if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateMid)
    {
      //分离模式
      target_yaw -= 0.003f * rc_ptr->rc_rh();
      target_yaw = restrict_angle(target_yaw);

      Car.setVel(10.0f * rc_ptr->rc_lv() * cos(theta) + 10.0f * rc_ptr->rc_lh() * sin(theta), 10.0f * rc_ptr->rc_lh() * cos(theta) - 10.0f * rc_ptr->rc_lv() * sin(theta), 0.0f);
    }
    else if (rc_ptr->rc_l_switch() == remote_control::kSwitchStateDown)
    {
      //陀螺模式
      target_yaw -= 0.003f * rc_ptr->rc_rh();
      target_yaw = restrict_angle(target_yaw);  

      Car.setVel(10.0f * rc_ptr->rc_lv() * cos(theta) + 10.0f * rc_ptr->rc_lh() * sin(theta), 10.0f * rc_ptr->rc_lh() * cos(theta) - 10.0f * rc_ptr->rc_lv() * sin(theta), 10.0f);
    }
    //更新电机数据
    YawMotor.setTargetYaw(target_yaw);
    YawMotor.update(POS_MODE);
    Car.update();
    data_to_H7[2] = 0x00;
    Send_To_H7();
  }
  else if (tick >= 1000 && (rc_ptr->rc_r_switch() == remote_control::kSwitchStateUp || rc_ptr->rc_r_switch() == remote_control::kSwitchStateDown))
  {
    // 急停
    uint8_t zero[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN_Send_Msg(&hcan2, zero, 0x200, 8);
    mit_ctrl(&hcan1, 0x003, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    data_to_H7[2] = 0xFF;
    target_yaw = imu_datas_H7.euler_vals[YAW];
    YawMotor.reset();
    Send_To_H7();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim == &htim6)
  {
    MainTask();
  }
}
uint8_t rx_data = 0;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    if (Size == remote_control::kRcRxDataLen)
    {
      //在这里刷新看门狗
      HAL_IWDG_Refresh(&hiwdg);
      rc_ptr->decode(rx_buf);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen);
  }
}
