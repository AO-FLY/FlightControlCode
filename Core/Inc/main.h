/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern uint8_t key_flag;
extern uint8_t USRAT_flag;
//extern int Height;
extern float T265_x, T265_y /*T265_z*/;
extern float /*T265x_speed, T265y_speed , T265z_speed,*/ Yaw_T265;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PE2_Pin GPIO_PIN_2
#define PE2_GPIO_Port GPIOE
#define PE3_Pin GPIO_PIN_3
#define PE3_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define KEY1_EXTI_IRQn EXTI0_IRQn
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOC
#define KEY2_EXTI_IRQn EXTI1_IRQn
#define KEY3_Pin GPIO_PIN_2
#define KEY3_GPIO_Port GPIOC
#define KEY3_EXTI_IRQn EXTI2_IRQn
#define KEY4_Pin GPIO_PIN_3
#define KEY4_GPIO_Port GPIOC
#define KEY4_EXTI_IRQn EXTI3_IRQn
#define WS2812B_Pin GPIO_PIN_1
#define WS2812B_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_7
#define BUZZER_GPIO_Port GPIOE
#define PE8_Pin GPIO_PIN_8
#define PE8_GPIO_Port GPIOE
#define NRF24L01_CS_Pin GPIO_PIN_15
#define NRF24L01_CS_GPIO_Port GPIOA
#define PD0_Pin GPIO_PIN_0
#define PD0_GPIO_Port GPIOD
#define Light_Pin GPIO_PIN_1
#define Light_GPIO_Port GPIOD
#define NRF24L01_CE_Pin GPIO_PIN_3
#define NRF24L01_CE_GPIO_Port GPIOD
#define NRF24L01_IRQ_Pin GPIO_PIN_4
#define NRF24L01_IRQ_GPIO_Port GPIOD
#define NRF24L01_SCK_Pin GPIO_PIN_3
#define NRF24L01_SCK_GPIO_Port GPIOB
#define NRF24L01_MISO_Pin GPIO_PIN_4
#define NRF24L01_MISO_GPIO_Port GPIOB
#define NRF24L01_MOSI_Pin GPIO_PIN_5
#define NRF24L01_MOSI_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define PE0_Pin GPIO_PIN_0
#define PE0_GPIO_Port GPIOE
#define PE1_Pin GPIO_PIN_1
#define PE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
