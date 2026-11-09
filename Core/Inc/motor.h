#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

// ================= 电机控制函数声明 =================

/**
 * @brief  停车
 */
void Car_Stop(void);

/**
 * @brief  电机基础前进 (开环)
 * @param  speed: PWM 占空比 (0-999)
 */
void Car_Forward(uint16_t speed);

/**
 * @brief  电机原地左转 (开环)
 * @param  speed: PWM 占空比 (0-999)
 */
void Car_TurnLeft(uint16_t speed);

/**
 * @brief  编码器闭环直行 (带梯形加减速)
 * @param  distance_cm: 目标距离 (单位: 厘米)
 */
void Car_Forward_Distance(float distance_cm);

/**
 * @brief  陀螺仪闭环左转 (带平滑减速)
 */
void Turn_Left_90(void);

#endif

