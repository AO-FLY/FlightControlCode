//
// Created by 28182 on 2023/7/29.
//

#ifndef PROJECT_OPTICAL_FLOW_H
#define PROJECT_OPTICAL_FLOW_H

#include "usart.h"

#define USART3_BOUND 500000	//串口2通信波特率

#define USART2_BUF_LEN 30	//串口2接收数据缓冲区长度

extern float DX_SPE,DY_SPE;	//惯导融合后的速度，单位cm/s
extern float DX_FIX,DY_FIX;	//修正后的X,Y轴移动速度，适用于积分运算
extern float DX_DIS,DY_DIS;
extern float DX_LONG,DY_LONG;
extern float DX_LONG_2,DY_LONG_2;
extern int Height;		//高度数据

/*
 * 函数声明
 */
void optical_flow_receive(uint8_t* array,uint8_t len);
uint8_t Memory_Usart2_Data(uint8_t data);		//缓存串口2数据
void Processing_Usart2_Data(void);	//处理串口2的数据


#endif //PROJECT_OPTICAL_FLOW_H
