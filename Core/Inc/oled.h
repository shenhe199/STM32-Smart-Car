#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <stdint.h>

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char chr);
void OLED_ShowString(uint8_t x, uint8_t y, char *str);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num); 

#endif

