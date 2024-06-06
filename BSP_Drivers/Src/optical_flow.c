//
// Created by Haoyee_yin���� on 2023/7/29.
//  feat :@����
//

#include "optical_flow.h"

float DX_SPE,DY_SPE;	//�ߵ��ںϺ���ٶȣ���λcm/s
float DX_FIX,DY_FIX;	//�������X,Y���ƶ��ٶȣ������ڻ�������
float DX_DIS,DY_DIS;    //���������λ������
float DX_LONG,DY_LONG;
float DX_LONG_2,DY_LONG_2;
int Height = 9;	//�߶����ݣ������ڵ���Ϊ9cm����


//���崮��2���ݻ�����������USART3_BUF_LEN
uint8_t USART2_RX_BUF[USART2_BUF_LEN];

//������DMA����
void optical_flow_receive(uint8_t* array,uint8_t len)
{
    for(int i=0;i<len;i++)
    {
        if(Memory_Usart2_Data(array[i]) == 1)
            break;
    }
}

/*
 * ��������Memory_Usart3_Data
 * ��  �ܣ�У�鴮�ڽ��յ����ݲ��洢�ڻ�����
 * ��  ����data -> ���ڽ��յ�����
 * ����ֵ����
 */
uint8_t Memory_Usart2_Data(uint8_t data)
{
    static uint8_t state=0, len=0, cnt=0;

    if(state==0 && data==0xAA)		//֡ͷ0xAA
    {

        USART2_RX_BUF[0] = data;
        state = 1;
    }
    else if(state==1 && data==0xFF)	//Ŀ���ַ0xFF
    {
        USART2_RX_BUF[1] = data;
        state = 2;
    }
    else if(state == 2)				//������
    {
        if(data == 0x51 || data == 0x34)
        {
            USART2_RX_BUF[2] = data;
            state = 3;
        }
        else state = 0;
    }
    else if(state == 3)				//���ݳ���
    {
        if(data == 0x0f || data == 0x07)
        {
            len = data + 2;
            state = 4;
            cnt = 4;
            USART2_RX_BUF[3] = data;
        }
        else state = 0;
    }
    else if(state == 4)				//��������
    {
        USART2_RX_BUF[cnt++] = data;
        if(cnt == len + 4)
            state = 5;
    }
    else if(state == 5)				//��������
    {
        state = 0;
        Processing_Usart2_Data();	//������յ�������
        return 1;
    }
    else    state = 0;				//һ֡���ս���

    return 0;
}

/*
 * ��������Processing_Usart3_Data
 * ��  �ܣ�������3���յĹ������ݣ��ó�����Ҫ������
 * ��  ������
 * ����ֵ����
 */
void Processing_Usart2_Data(void)
{
    uint8_t sumcheck = 0,addcheck = 0;
    int Height_t;

    for(uint8_t i=0;i<(USART2_RX_BUF[3] + 4); i++)	//�����ۼӽ��к�У��
    {
        sumcheck += USART2_RX_BUF[i];
        addcheck += sumcheck;
    }
    if(sumcheck == USART2_RX_BUF[USART2_RX_BUF[3] + 4] && addcheck == USART2_RX_BUF[USART2_RX_BUF[3] + 5])
    {
        if(USART2_RX_BUF[3] == 0x0F)			//�ж��Ƿ�Ϊλ������
        {
            DX_SPE = (float)*(short *)&USART2_RX_BUF[6];
            DY_SPE = (float)*(short *)&USART2_RX_BUF[8];
            DX_FIX = (float)*(short *)&USART2_RX_BUF[10];
            DY_FIX = (float)*(short *)&USART2_RX_BUF[12];
            DX_DIS = (float)*(short *)&USART2_RX_BUF[14];
            DY_DIS = (float)*(short *)&USART2_RX_BUF[16];
            DX_LONG += DX_FIX*0.006f;	//����X����λ��
            DY_LONG += DY_FIX*0.006f;	//����Y����λ��
            DX_LONG_2 += DX_SPE*0.005f;	//����X����λ��
            DY_LONG_2 += DY_SPE*0.05f;	//����Y����λ��
        }
        else if(USART2_RX_BUF[3] == 0x07)
        {
            //�߶����ݽ���
            Height_t = (int)*(int *)&USART2_RX_BUF[7];
            if(Height_t < 240 && Height_t > 0)	//�ų��������
                Height = Height_t;
            //printf("Height = %d\r\n",Height);
        }
    }
    else;
//        printf("���ݽ���ʧ�ܡ�");
}



