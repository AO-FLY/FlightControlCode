#ifndef PROJECT_WS2812B_H
#define PROJECT_WS2812B_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

#include "Delay.h"

/*
软件模拟时序
需要
    1.GPIO
        开漏浮空输出模式，需要外接5V上拉，注意需要选择手册中标有FT的管脚
        将其设置为最高规格输出
*/
#define WS2812_GPIO_Group GPIOA
#define WS2812_GPIO_Pin GPIO_PIN_1
// #define WS2812_Software

/*
硬件定时器PWM+DMA:
需要:
    1.定时器:
        PWM输出一个通道
        不分频
        计数值为 1.25us(公式: 1.25 *系统频率(单位MHz))
        输出IO设置为开漏浮空输出(外接5V上拉)
    2.DMA
        内存到外设
        字(word)模式
        开启DMA中断
0码的是 0.4us,1码是0.8us
公式是 t(us)*系统频率(单位MHz)
 */
//extern TIM_HandleTypeDef htim2;
//#define WS2812_Hardware
#define WS2812_TIM htim2
#define WS2812_TIM_Channel TIM_CHANNEL_2
#define WS2812_Code_0 (32u)
#define WS2812_Code_1 (71u)

#define WS2812_Num 1

extern uint32_t WS2812_Data[WS2812_Num];
void WS2812_Code_Reast(void);
void WS2812_Send(void);

void WS2812_Start(void);
void delay_ns(uint32_t ns);
void delay_04us(void);
void delay_08us(void);

#define T0H() delay_04us()
#define T0L() delay_08us()
#define T1H() delay_08us()
#define T1L() delay_04us()

void RGB_On(void);
void RGB_Off(void);
void RGB_Start(void);
void Ws2812_Send(uint8_t data);
void ws218_Clear(void);
void RGB_Lighting(uint8_t *str);
void RGB_Light_Red(void);
void RGB_Light_Green(void);
void RGB_Light_Blue(void);
void RGB_Light_Purple(void);
void RGB_Light_Pink(void);
void RGB_Light_Yellow(void);
void RGB_Light_Orange(void);

#endif //PROJECT_WS2812B_H
