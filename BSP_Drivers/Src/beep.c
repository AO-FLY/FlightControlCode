#include "beep.h"
#include "cmsis_os.h"

void beep_on()  //ԭ����E BUZZER_Pin
{
    HAL_GPIO_WritePin(GPIOD, Light_Pin, 0); // ��������
//    HAL_Delay(50);
    osDelay(50);
    HAL_GPIO_WritePin(GPIOD, Light_Pin, 1);
    osDelay(120);
}

void beep_off()
{
    HAL_GPIO_WritePin(GPIOD, Light_Pin, 0);
}

