//
// Created by 28182 on 2023/7/29.
//

#ifndef PROJECT_OPTICAL_FLOW_H
#define PROJECT_OPTICAL_FLOW_H

#include "usart.h"

#define USART3_BOUND 500000	//����2ͨ�Ų�����

#define USART2_BUF_LEN 30	//����2�������ݻ���������

extern float DX_SPE,DY_SPE;	//�ߵ��ںϺ���ٶȣ���λcm/s
extern float DX_FIX,DY_FIX;	//�������X,Y���ƶ��ٶȣ������ڻ�������
extern float DX_DIS,DY_DIS;
extern float DX_LONG,DY_LONG;
extern float DX_LONG_2,DY_LONG_2;
extern int Height;		//�߶�����

/*
 * ��������
 */
void optical_flow_receive(uint8_t* array,uint8_t len);
uint8_t Memory_Usart2_Data(uint8_t data);		//���洮��2����
void Processing_Usart2_Data(void);	//������2������


#endif //PROJECT_OPTICAL_FLOW_H
