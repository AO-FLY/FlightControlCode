/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "retarget.h"

//uint8_t res_buf = 0;
uint8_t optical_flow_buf[34] = {0};
uint8_t T265_buf[14] = {0};  //T265接收数据缓冲区
uint8_t Car_buf[5] = {0};  //小车发送缓冲区

//ͨ��Э��֡ͷ ֡β
const unsigned char header[2]  = {0xfd, 0x88};
//const unsigned char ender[2]   = {0x0d, 0x0a};

union receivedata
{
    float d;
    unsigned char data[4];
}t265_x,t265_y,t265_z,t265_speed_x,t265_speed_y,/*t265_speed_z*/ yaw_T265;


/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
//  RetargetInit(&huart1);  //串口1使用重定�????
  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 576000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void fsprintf(char * stra, float x, uint8_t flen)
{
    uint32_t base;
    int64_t dn;
    char mc[32];

    base = pow(10,flen);
    dn = x*base;
    sprintf(stra, "%d.", (int)(dn/base));
    dn = abs(dn);
    if(dn%base==0)
    {
        for(uint8_t j=1;j<=flen;j++)
        {
            stra = strcat(stra, "0");
        }
        return;
    }
    else
    {
        if(flen==1){
            sprintf(mc, "%d", (int)(dn%base));
            stra = strcat(stra, mc);
            return;
        }

        for(uint8_t j=1;j<flen;j++)
        {
            if((dn%base)<pow(10,j))
            {
                for(uint8_t k=1;k<=(flen-j);k++)
                {
                    stra = strcat(stra, "0");
                }
                sprintf(mc, "%d", (int)(dn%base));
                stra = strcat(stra, mc);
                return;
            }
        }
        sprintf(mc, "%d", (int)(dn%base));
        stra = strcat(stra, mc);
        return;
    }
}

/*
 * ��������Vofa_Input
 * ��  �ܣ�װ��Ҫ���͵�������������
 * ��  ����data -> ����		channel -> ͨ��
 * ����ֵ��
 */
void Vofa_Input(float data,unsigned char channel)
{
    if(channel < ChannNum)
        Vofa_Sbuffer.Date[channel] = data;
}

/*
 * ��������Vofa_Send
 * ��  �ܣ�����������vofa��λ��
 * ��  ������
 * ����ֵ����
 */
void Vofa_Send(void)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&Vofa_Sbuffer, sizeof(Frame), 50);
}

float Position_filtering(float input,float* input_pre)
{
    float output;

//    if(input - *input_pre > 3)
//    {
//        input = (float)*input_pre;
//    }

    output = input * 0.2f + *input_pre * 0.8f;
    *input_pre = input;

    return output;
}

/**************************************************************************
�������ܣ�ͨ�������жϷ���������ȡ��λ�����͵������ֿ����ٶȡ�Ԥ�����Ʊ�־λ���ֱ���������
��ڲ�����t265��x�ᣬy�ᣬz��
����  ֵ��0
����֡�ĸ�ʽ��0xfd 0x88 size 00 00 ... 00 00 0x0d 0x0a
0xfd 0x88:֡ͷ 0x0d 0x0a֡β��
**************************************************************************/
//void UsartReceiveOneData(float *t265x,float *t265y,float *t265z)
//{
////    unsigned char USART_Receiver              = 0;          //��������
//    static unsigned char USARTBufferIndex     = 0;
//    static short j=0,k=0;
//    static unsigned char USARTReceiverFront   = 0;
//    static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
//    static short dataLength                   = 0;
//
////    HAL_UART_Receive(&huart3,&USART_Receiver,1,0xff);
//
//    //������Ϣͷ
//    if(Start_Flag == START)
//    {
//        if(USART_Receiver == 0x88)                           //buf[1]
//        {
//            if(USARTReceiverFront == 0xfd)        //����ͷ��λ //buf[0]
//            {
//                Start_Flag = !START;              //�յ�����ͷ����ʼ��������
//                //printf("header ok\n");
//                receiveBuff[0]=header[0];         //buf[0]
//                receiveBuff[1]=header[1];         //buf[1]
//                USARTBufferIndex = 0;             //��������ʼ��
//            }
//        }
//        else
//        {
//            USARTReceiverFront = USART_Receiver;
//        }
//    }
//    else
//    {
//        switch(USARTBufferIndex)
//        {
//            case 0://����t265�������ݵĳ���
//                receiveBuff[2] = USART_Receiver;
//                dataLength     = receiveBuff[2];            //buf[2]
//                USARTBufferIndex++;
//                break;
//            case 1://��������xyz���ݲ���ֵ����
//                receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[15]
//                j++;
//                if(j >= dataLength)
//                {
//                    j = 0;
//                    USARTBufferIndex++;
//                }
//                break;
//
//            case 2://������Ϣβ
//                if(k==0)
//                {
//                    //����0d     buf[16]  �����ж�
//                    k++;
//                }
//                else if (k==1)
//                {
//                    //����0a     buf[17] �����ж�
//
//                    //�����ٶȸ�ֵ����
//                    for(k = 0; k < 4; k++)
//                    {
//                        t265_x.data[k] = receiveBuff[k + 3]; //buf[3] buf[4] buf[5] buf[6]
//                        t265_y.data[k] = receiveBuff[k + 7]; //buf[7] buf[8] buf[9] buf[10]
//                        t265_z.data[k] = receiveBuff[k + 11]; //buf[11] buf[12] buf[13] buf[14]
//                    }
////                    USART2_Send_String(t265_x.data,4);
//                    //���긳ֵ����
//
//                    *t265x = t265_x.d;
//                    *t265y = t265_y.d;
//                    *t265z = t265_z.d;
//
//                    USARTReceiverFront = 0;
//                    Start_Flag         = START;
//                    dataLength         = 0;
//                    j = 0;
//                    k = 0;
//                }
//                break;
//            default:break;
//        }
//    }
//}

//void UsartReceiveOneData(float *t265x,float *t265y,float *t265z,float *t265x_speed,float *t265y_speed,float *t265z_speed)

void UsartReceiveOneData(float *t265x,float *t265y,float *t265_yaw)
{
//    if(T265_buf[0]==0xfd)
//    {
//        HAL_UART_Receive_IT(&huart3,T265_buf+1,1);
//        if(T265_buf[1]==0x88)
//        {
////            printf("YES!!\r\n");
////            return ;
////            HAL_UART_Receive(&huart3, T265_buf + 2, 12, 0xffff);
//            HAL_UART_Receive_DMA(&huart3, T265_buf + 2, 12);

//            static float t265x_pre;

            if(T265_buf[0] == 0xfd && T265_buf[1] == 0x88) {

                //�����ٶȸ�ֵ����
                for (int k = 0; k < 4; k++) {
                    t265_x.data[k] = T265_buf[k + 2]; //buf[2] buf[3] buf[4] buf[5]
                    t265_y.data[k] = T265_buf[k + 6]; //buf[6] buf[7] buf[8] buf[9]
//                t265_z.data[k] = T265_buf[k + 10]; //buf[10] buf[11] buf[12] buf[13]
//                t265_speed_x.data[k] = T265_buf[k + 14]; //buf[14] buf[15] buf[16] buf[17]
//                t265_speed_y.data[k] = T265_buf[k + 18]; //buf[18] buf[19] buf[20] buf[21]
//                t265_speed_z.data[k] = T265_buf[k + 22]; //buf[22] buf[23] buf[24] buf[25]
                    yaw_T265.data[k] = T265_buf[k + 10];  //buf[10] buf[11] buf[12] buf[13]
                }

                //���긳ֵ����
                if (t265_x.d < 200 && t265_x.d > -200) *t265x = t265_x.d;
                if (t265_y.d < 200 && t265_y.d > -200) *t265y = t265_y.d;
//            if (t265_z.d < 200 && t265_z.d > -200) *t265z = t265_z.d;
//            if (t265_x.d < 100 && t265_x.d > -100) *t265x_speed = t265_speed_x.d;
//            if (t265_y.d < 100 && t265_y.d > -100) *t265y_speed = t265_speed_y.d;
//            if (t265_z.d < 100 && t265_z.d > -100) *t265z_speed = t265_speed_z.d;
                if (yaw_T265.d < 182 && yaw_T265.d > -182) *t265_yaw = yaw_T265.d;

            }

//            printf("YES!!\r\n");
//        }
//        else
//            HAL_UART_Receive_IT(&huart3,T265_buf,1);
//    }
//    else
//    {
//        T265_buf[0] = 0;
//
//    }
}


//发送给小车，包括飞近的位置信息和火焰位置信息
void Sending_car(int self_x,int self_y,int fire_x,int fire_y)
{
//    self_position_x.d = self_x;
//    self_position_y.d = self_y;
//    fire_position_x.d = fire_x;
//    fire_position_y.d = fire_y;
//
//    for (int j = 0; j < 4; j++)
//    {
//        Car_buf[j] = self_position_x.data[j];
//        Car_buf[j+4] = self_position_y.data[j];
//        Car_buf[j+8] = fire_position_x.data[j];
//        Car_buf[j+12] = fire_position_y.data[j];
//    }

    //上方注释部分为旧协议，以下方式更精简且传输准确
    Car_buf[0] = (uint8_t)self_x;
    Car_buf[1] = (uint8_t)self_y;
    Car_buf[2] = (uint8_t)fire_x;
    Car_buf[3] = (uint8_t)fire_y;
    Car_buf[4] = '\n';

//    HAL_UART_Transmit(&huart1, (uint8_t *) &self_y, 1, 50);
    HAL_UART_Transmit_IT(&huart1, Car_buf, 5);
}

#ifdef __GNUC__ // 重定向printf
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);
    return ch;
}
/* USER CODE END 1 */
