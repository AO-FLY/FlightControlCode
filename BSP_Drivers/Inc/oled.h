#ifndef STM32F407VGT6_OLED_H
#define STM32F407VGT6_OLED_H

#include "stdlib.h"
#include "main.h"
#include "i2c.h"

void WriteCmd();
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);
void OLED_Init();
void OLED_Clear();
void OLED_Display_On();
void OLED_Display_Off();
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_On();
unsigned int oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no);

#endif //STM32F407VGT6_OLED_H
