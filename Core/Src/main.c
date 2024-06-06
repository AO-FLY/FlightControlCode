/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "JY901.h"
#include "retarget.h"
#include "oled.h"
#include "ws2812b.h"
#include "beep.h"
#include "optical_flow.h"
//#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)  //ws2812b
//{
//    printf("DMA_input\r\n");
//    WS2812_Send();
//    printf("DMA_send!\r\n");
//}
uint8_t key_flag = 0;
uint8_t USRAT_flag = 0;
//int Height = 0;
float T265_x = 0, T265_y = 0/*, T265_z = 0*/;
/*float T265x_speed = 0, T265y_speed = 0, T265z_speed = 0*/;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//    HAL_TIM_PWM_Init(&htim1);  //PWM初始�????

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
//  while(JY901_check())
//  {
//      printf("JY901 ERROR!\r");
//  }
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1); // ͨ��1 PWM��ʼ��
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2); // ͨ��2 PWM��ʼ��
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3); // ͨ��3 PWM��ʼ��
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); // ͨ��4 PWM��ʼ��
//    HAL_UART_Receive_IT(&huart1, &USRAT_flag, 1);
//    HAL_UART_Receive_DMA(&huart2,optical_flow_buf,34);  //这里是光流数据！！！
//    HAL_UART_Receive_DMA(&huart3, T265_buf, 14);      //这里是T265数据！！！
//    HAL_UART_Receive_DMA(&huart1, Car_buf, 5);

    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4); // 舱门舵机控制
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 500);    //舱门关

    HAL_GPIO_WritePin(GPIOD, Light_Pin, 1); //新的蜂鸣器关

    OLED_Init();
    OLED_Clear();

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //以下代码均为调试期间的裸机调试代码，可忽略

//      Get_Angle_Data(&pitch,&roll,&Yaw_T265);
//      printf("%f %f %f \r",pitch,roll,Yaw_T265);
//      HAL_Delay(100);
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);
//      WS2812_Start();
//      RGB_Start();
//        RGB_On();
//      delay_08us();
//        RGB_Off();
//      delay_08us();
//      RGB_Light_Red();
//      HAL_Delay(1000);
//      RGB_Light_Blue();

//      HAL_UART_Transmit(&huart3, &USART_Receiver, 1, 0xff);
//      HAL_Delay(100);

//      beep_on();beep_on();beep_on();
//      ws218_Clear();

//      HAL_Delay(1000);
//      RGB_Off();

//      RGB_Off();
//      HAL_Delay(10);
//      WS2812_Send();
//      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);
//      HAL_Delay(100);
//      optical_flow_receive(optical_flow_buf,60);

//      HAL_UART_Transmit(&huart1, optical_flow_buf, 120, 0xffff);
//      HAL_Delay(100);
//      HAL_UART_Receive_DMA(&huart2,optical_flow_buf,120);
//      HAL_Delay(100);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口中断回调。为防止硬件中断干扰操作系统时序，已采用空闲中断
{
//    if(huart->Instance == USART1)   //
//    {
//        printf("hello!\r\n");
//        HAL_UART_Receive_IT(&huart1, &USRAT_flag, 1);
////        key_flag = 1;
//    }
//    if(huart->Instance == USART2)
//    {
//        if((int)TF_buf[0]==0x59)
//        {
//            HAL_UART_Receive(&huart2,TF_buf+1,1,0xff);
//            if((int)TF_buf[1]==0x59)
//            {
//                HAL_UART_Receive(&huart2,TF_buf+2,7, 0xff);
//                check=(int)TF_buf[0]+(int)TF_buf[1]+(int)TF_buf[2]+(int)TF_buf[3]+(int)TF_buf[4]+(int)TF_buf[5]+(int)TF_buf[6]+(int)TF_buf[7];
//                if((int)TF_buf[8]==(check&0xff))
//                {
//                    if(((int)TF_buf[2]+(int)TF_buf[3]*256) > 0 && ((int)TF_buf[2]+(int)TF_buf[3]*256) < 240)
//                    {
//                        Height = (int)TF_buf[2]+(int)TF_buf[3]*256;
//                    }
//                }
//            }
//        }
//        else
//            TF_buf[0] = 0;
//        HAL_UART_Receive_IT(&huart2,TF_buf,1);
//    }

//    if(huart->Instance == USART3)   //�жϷ��������жϵĴ���
//    {
////        printf("Hello!T265!\r\n");
//        HAL_UART_Receive_DMA(&huart3, T265_buf, 14);
////          HAL_UART_Receive_DMA(&huart3, T265_buf + 2, 12);
//    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == KEY1_Pin) {
//        printf("key_input\r\n");
        key_flag = 1;
    }
    if (GPIO_Pin == KEY2_Pin)
    {
//        printf("key_input\r\n");
        key_flag = 2;
    }
    if (GPIO_Pin == KEY3_Pin)
    {
//        printf("key_input\r\n");
        key_flag = 3;
    }
    if (GPIO_Pin == KEY4_Pin)
    {
//        printf("key_input\r\n");
        key_flag = 4;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1250);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1250);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1250);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1250);

      beep_on();beep_on();beep_on();

      HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

