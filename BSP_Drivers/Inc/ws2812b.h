#ifndef PROJECT_WS2812B_H
#define PROJECT_WS2812B_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "tim.h"

#include "Delay.h"

/*
���ģ��ʱ��
��Ҫ
    1.GPIO
        ��©�������ģʽ����Ҫ���5V������ע����Ҫѡ���ֲ��б���FT�Ĺܽ�
        ��������Ϊ��߹�����
*/
#define WS2812_GPIO_Group GPIOA
#define WS2812_GPIO_Pin GPIO_PIN_1
// #define WS2812_Software

/*
Ӳ����ʱ��PWM+DMA:
��Ҫ:
    1.��ʱ��:
        PWM���һ��ͨ��
        ����Ƶ
        ����ֵΪ 1.25us(��ʽ: 1.25 *ϵͳƵ��(��λMHz))
        ���IO����Ϊ��©�������(���5V����)
    2.DMA
        �ڴ浽����
        ��(word)ģʽ
        ����DMA�ж�
0����� 0.4us,1����0.8us
��ʽ�� t(us)*ϵͳƵ��(��λMHz)
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
