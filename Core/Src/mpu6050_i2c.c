#include "mpu6050_i2c.h"
#include "main.h"

#define SCL_H()   HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN)

//微秒级延时 
static inline void IIC_Delay(void)
{
    volatile uint32_t i = 30;
    while (i--) __NOP();
}

//初始化
void IIC_Init(void)
{
    SCL_H();
    SDA_H();
}

/* SDA 被卡死时调用：发 9 个时钟把从机状态机拉出来 */
void IIC_BusRecover(void)
{
    SDA_H();//释放 SDA
    for (int i = 0; i < 9; i++) {
        SCL_H(); IIC_Delay();//拉高 SCL
        SCL_L(); IIC_Delay();//拉低 SCL
    }
    IIC_Stop();
}

//开始
void IIC_Start(void)
{
    SDA_H(); IIC_Delay();//拉高 SDA，准备发起 START
    SCL_H(); IIC_Delay();//拉高 SCL，准备发起 START
    SDA_L(); IIC_Delay();// SCL 高时 SDA 下降 = START
    SCL_L(); IIC_Delay();//收尾拉低 SCL：保证后面可直接再来 Start（Restart）
}

//停止
void IIC_Stop(void)
{
    SCL_L(); IIC_Delay();//拉低 SCL，准备改变 SDA
    SDA_L(); IIC_Delay();//拉低 SDA，准备发起 STOP
    SCL_H(); IIC_Delay();//拉高 SCL，准备发起 STOP
    SDA_H(); IIC_Delay();// SCL 高时 SDA 上升 = STOP
}

//发送
uint8_t IIC_Send_Byte(uint8_t byte)
{
    uint8_t ack;
    //发送 8 位数据
    for (int i = 0; i < 8; i++) {
        SCL_L(); IIC_Delay();//拉低 SCL，准备改变 SDA 
        if (byte & 0x80) SDA_H(); else SDA_L();//根据最高位发送 SDA
        byte <<= 1;//左移一位，准备发送下一位
        IIC_Delay();
        SCL_H(); IIC_Delay();//拉高 SCL，数据被从机采样
    }
    /* 第 9 个时钟：读从机 ACK */
    SCL_L(); IIC_Delay();//拉低 SCL，准备读 ACK
    SDA_H(); IIC_Delay();//释放 SDA，准备读 ACK
    SCL_H(); IIC_Delay();//拉高 SCL，从机输出 ACK
    ack = SDA_READ();// 0=ACK，1=NACK
    SCL_L(); IIC_Delay();//收尾拉低 SCL：保证后面可直接再来 Start（Restart）
    return ack;
}

//读取
uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t byte = 0;
    SDA_H();//释放 SDA，准备读数据
    for (int i = 0; i < 8; i++) {
        SCL_L(); IIC_Delay();//拉低 SCL，准备读数据
        SCL_H(); IIC_Delay();//拉高 SCL，从机输出数据
        byte <<= 1;//左移一位，准备接收下一位
        if (SDA_READ()) byte |= 0x01;//读到 1，置位最低位
    }
    /* 第 9 个时钟：主机应答 */
    SCL_L(); IIC_Delay();//拉低 SCL，准备发 ACK/NACK
    if (ack) SDA_L(); else SDA_H();//根据参数 ack 发送 ACK/NACK
    IIC_Delay();
    SCL_H(); IIC_Delay();//拉高 SCL，从机采样 ACK/NACK
    SCL_L(); IIC_Delay();//收尾拉低 SCL：保证后面可直接再来 Start（Restart）
    SDA_H();//释放 SDA，准备下一次读写
    return byte;
}
