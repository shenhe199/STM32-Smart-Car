/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//================== MPU6050 =====================
#define GYRO_SENSITIVITY  16.4f       // 灵敏度 
#define GYRO_DEADBAND     0.5f        // 【死区过滤】角速度死区(度/秒)
#define STRAIGHT_DEADBAND 1.0f        // 【直行纠偏】直行偏差死区(度)
#define STRAIGHT_CORRECT  25          // 【直行纠偏】直行纠偏力度/1000
#define ALIGN_DEADBAND    0.7f        // 【航向对齐】航向对齐死区（度）
#define MIN_SPEED         60.0f       // 【航向对齐】最小速度/1000 
/* #define ANGLE_INTEGRAL_K  0.017f      // 时间系数 */

// ================= 编码器 =================
#define ENCODER_PER_CM    70.0f       // 每cm的脉冲 

// ================= FORWARD =================
#define STRAIGHT_DIST_CM  58.5f       // 设定距离(cm)
#define MOTOR_MAX_SPEED   200         // 最大速度/1000
#define MOTOR_MIN_SPEED   100         // 最小速度/1000
#define ACCEL_RATIO       0.25f       // 加速区占总距离的比例 (%)
#define DECEL_RATIO       0.25f       // 减速区占总距离的比例 (%)
/* #define MOTOR_K           1.09f        // 轮速差硬件补偿系数 */ 

// ================= TURNING =================
#define TURN_TARGET_ANGLE 84.0f       // 设定角度 (90度)
#define TURN_MAX_SPEED    120.0f      // 最大速度/1000
#define TURN_MIN_SPEED    80.0f       // 最小速度/1000
#define TURN_ACCEL_ANGLE  20.0f       // 加速区占总距离的比例 (%)
#define TURN_DECEL_ANGLE  45.0f       // 减速区占总距离的比例 (%)

//=======================================================

extern float gz_offset;//陀螺仪零点漂移校准值
extern volatile float current_angle;//当前角度 
extern volatile float real_gz;//真实角速度 

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

