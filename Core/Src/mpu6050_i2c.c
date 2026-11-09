#include "mpu6050_i2c.h"
#include "main.h"   /* HAL 头 */

#define SCL_H()   HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(IIC_SCL_GPIO_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ() HAL_GPIO_ReadPin(IIC_SDA_GPIO_PORT, IIC_SDA_PIN)

static inline void IIC_Delay(void)
{
    /* ~100kHz，72MHz 下约 5us。按主频微调，或改用 DWT/定时器 */
    volatile uint32_t i = 30;
    while (i--) __NOP();
}

void IIC_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = IIC_SCL_PIN | IIC_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏关键：SDA 天然可读回 */
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &g);
    SCL_H(); SDA_H();
}

/* SDA 被卡死时调用：发 9 个时钟把从机状态机拉出来 */
void IIC_BusRecover(void)
{
    SDA_H();
    for (int i = 0; i < 9; i++) {
        SCL_H(); IIC_Delay();
        SCL_L(); IIC_Delay();
    }
    IIC_Stop();
}

void IIC_Start(void)
{
    SDA_H(); IIC_Delay();
    SCL_H(); IIC_Delay();
    SDA_L(); IIC_Delay();   /* SCL 高时 SDA 下降 = START */
    SCL_L(); IIC_Delay();   /* 收尾拉低 SCL：保证后面可直接再来 Start（Restart） */
}

void IIC_Stop(void)
{
    SCL_L(); IIC_Delay();
    SDA_L(); IIC_Delay();
    SCL_H(); IIC_Delay();
    SDA_H(); IIC_Delay();   /* SCL 高时 SDA 上升 = STOP */
}

uint8_t IIC_Send_Byte(uint8_t byte)
{
    uint8_t ack;
    for (int i = 0; i < 8; i++) {
        SCL_L(); IIC_Delay();
        if (byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1;
        IIC_Delay();
        SCL_H(); IIC_Delay();
    }
    /* 第 9 个时钟：读从机 ACK */
    SCL_L(); IIC_Delay();
    SDA_H(); IIC_Delay();       /* 释放 SDA */
    SCL_H(); IIC_Delay();
    ack = SDA_READ();           /* 0=ACK，1=NACK */
    SCL_L(); IIC_Delay();       /* 拉低 SCL 收尾 */
    return ack;
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t byte = 0;
    SDA_H();                    /* 主机释放 SDA */
    for (int i = 0; i < 8; i++) {
        SCL_L(); IIC_Delay();
        SCL_H(); IIC_Delay();
        byte <<= 1;
        if (SDA_READ()) byte |= 0x01;
    }
    /* 第 9 个时钟：主机应答 */
    SCL_L(); IIC_Delay();
    if (ack) SDA_L(); else SDA_H();   /* ACK 拉低 / NACK 保持高 */
    IIC_Delay();
    SCL_H(); IIC_Delay();
    SCL_L(); IIC_Delay();
    SDA_H();                    /* 释放 SDA */
    return byte;
}
