#ifndef __MPU6050_I2C_H
#define __MPU6050_I2C_H
#include <stdint.h>


#define IIC_SCL_GPIO_PORT   GPIOB
#define IIC_SCL_PIN         GPIO_PIN_14
#define IIC_SDA_GPIO_PORT   GPIOB
#define IIC_SDA_PIN         GPIO_PIN_15


#define MPU6050_ADDR        0x68
#define MPU6050_ADDR_W      ((MPU6050_ADDR << 1) | 0)
#define MPU6050_ADDR_R      ((MPU6050_ADDR << 1) | 1)

void    IIC_Init(void);
void    IIC_BusRecover(void);
void    IIC_Start(void);
void    IIC_Stop(void);
uint8_t IIC_Send_Byte(uint8_t byte);
uint8_t IIC_Read_Byte(uint8_t ack);

#endif

