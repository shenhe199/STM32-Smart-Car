#include "mpu6050.h"
#include "mpu6050_i2c.h"
#include "main.h"

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

volatile int16_t ax = 0, ay = 0, az = 0;
volatile int16_t gx = 0, gy = 0, gz = 0;
volatile int16_t temp_raw = 0;

/* 返回 0=成功；1=地址NACK；2=寄存器NACK；3=数据NACK */
uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    uint8_t ret = 0;
    IIC_Start();
    if (IIC_Send_Byte(MPU6050_ADDR_W)) ret = 1;
    else if (IIC_Send_Byte(reg))       ret = 2;
    else if (IIC_Send_Byte(data))      ret = 3;
    IIC_Stop();
    return ret;
}

uint8_t MPU6050_Read_Reg(uint8_t reg)
{
    uint8_t data;
    IIC_Start();
    IIC_Send_Byte(MPU6050_ADDR_W);
    IIC_Send_Byte(reg);
    IIC_Start();                        /* Restart */
    IIC_Send_Byte(MPU6050_ADDR_R);
    data = IIC_Read_Byte(0);            /* 单字节：必须 NACK */
    IIC_Stop();
    return data;
}

uint8_t MPU6050_Init(void)
{
    HAL_Delay(100);                                 /* 上电稳定 */

    if (MPU6050_Read_Reg(REG_WHO_AM_I) != 0x68)     /* 探测 */
        return 1;

    if (MPU6050_Write_Reg(REG_PWR_MGMT_1, 0x01))    /* 唤醒 + X轴陀螺PLL */
        return 2;
    HAL_Delay(50);

    MPU6050_Write_Reg(REG_SMPLRT_DIV,   0x09);      /* 1kHz/10 = 100Hz */
    MPU6050_Write_Reg(REG_CONFIG,       0x03);      /* DLPF 44Hz/42Hz */
    MPU6050_Write_Reg(REG_GYRO_CONFIG,  0x18);      /* ±2000°/s */
    MPU6050_Write_Reg(REG_ACCEL_CONFIG, 0x00);      /* ±2g  */
    HAL_Delay(10);
    return 0;
}

void MPU6050_Read_All(void)
{
    uint8_t buf[14];

    IIC_Start();
    if (IIC_Send_Byte(MPU6050_ADDR_W))              { IIC_Stop(); return; }
    if (IIC_Send_Byte(REG_ACCEL_XOUT_H))            { IIC_Stop(); return; }
    IIC_Start();
    if (IIC_Send_Byte(MPU6050_ADDR_R))              { IIC_Stop(); return; }

    /* 前 13 字节发 ACK(1)，最后 1 字节发 NACK(0) */
    for (int i = 0; i < 14; i++)
        buf[i] = IIC_Read_Byte(i == 13 ? 0 : 1);
    IIC_Stop();

    /* 严谨的有符号拼接 */
    ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
    temp_raw = (int16_t)(((uint16_t)buf[6] << 8) | buf[7]);
    gx = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
}

void MPU6050_Read_Accel(void)
{
    MPU6050_Read_All();     /* 保留旧接口，内部走连续读 */
}

float MPU6050_GetTempC(void)
{
    return temp_raw / 340.0f + 36.53f;
}
