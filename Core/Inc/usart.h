/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define START   0X11

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
#define ChannNum	4		//vofa上位机通道

typedef struct
{
    float Date[ChannNum];			//vofa上位机数据部分
    const unsigned char FramEnd[4];	//帧尾部分
}Frame;								//发送该结构图，完成波形显示

static Frame Vofa_Sbuffer = {{0},{0x00,0x00,0x80,0x7f}};	//帧尾格式

void fsprintf(char * stra, float x, uint8_t flen);
void Vofa_Input(float data,unsigned char channel);
void Vofa_Send(void);

void UsartReceiveOneData(float *t265x,float *t265y,float *t265_yaw);
float Position_filtering(float input,float* input_pre);

void Sending_car(int self_x,int self_y,int fire_x,int fire_y);

extern uint8_t optical_flow_buf[34];
extern uint8_t T265_buf[14];
extern uint8_t Car_buf[5];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

