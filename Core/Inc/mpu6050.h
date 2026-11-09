#ifndef __MPU6050_H
#define __MPU6050_H
#include <stdint.h>

extern volatile int16_t ax, ay, az;
extern volatile int16_t gx, gy, gz;
extern volatile int16_t temp_raw;


uint8_t MPU6050_Init(void);
uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read_Reg (uint8_t reg);
void    MPU6050_Read_All (void);
void    MPU6050_Read_Accel(void);

float   MPU6050_GetTempC(void);

#endif

