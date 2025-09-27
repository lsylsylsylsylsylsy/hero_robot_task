/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-09-07 17:57:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-09-24 20:06:14
 * @FilePath: \H7_project_1\Resources\Inc\HW_fdcan.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _HW_FDCAN_H_
#define _HW_FDCAN_H_
/* ------------------------------ Include ------------------------------ */
#include "stm32h723xx.h"
#include "stm32h7xx.h"
#include "system_user.hpp"

#include "fdcan.h"
/* ------------------------------ Macro Definition
 * ------------------------------ */
#define MAX_PITCH_ANGLE 3.0f
#define MIN_PITCH_ANGLE -3.0f
#define MAX_PITCH_VEL 2.0f
#define MIN_PITCH_VEL -2.0f
/* ------------------------------ Type Definition ------------------------------
 */

/* ------------------------------ Extern Global Variable
 * ------------------------------ */

/* ------------------------------ Function Declaration (used in other .c files)
 * ------------------------------ */

void FdcanFilter_Init(FDCAN_HandleTypeDef *hfdcan);

void FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                    uint8_t len);

#endif
