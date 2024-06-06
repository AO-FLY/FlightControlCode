//
// Created by Haoyee_yin奉曦 on 2023/7/29.
//  feat :@假人
//

#include "optical_flow.h"

float DX_SPE,DY_SPE;	//惯导融合后的速度，单位cm/s
float DX_FIX,DY_FIX;	//修正后的X,Y轴移动速度，适用于积分运算
float DX_DIS,DY_DIS;    //光流输出的位移数据
float DX_LONG,DY_LONG;
float DX_LONG_2,DY_LONG_2;
int Height = 9;	//高度数据，放置在地面为9cm左右


//定义串口2数据缓存区，长度USART3_BUF_LEN
uint8_t USART2_RX_BUF[USART2_BUF_LEN];

//适用于DMA接收
void optical_flow_receive(uint8_t* array,uint8_t len)
{
    for(int i=0;i<len;i++)
    {
        if(Memory_Usart2_Data(array[i]) == 1)
            break;
    }
}

/*
 * 函数名：Memory_Usart3_Data
 * 功  能：校验串口接收的数据并存储在缓存区
 * 参  数：data -> 串口接收的数据
 * 返回值：无
 */
uint8_t Memory_Usart2_Data(uint8_t data)
{
    static uint8_t state=0, len=0, cnt=0;

    if(state==0 && data==0xAA)		//帧头0xAA
    {

        USART2_RX_BUF[0] = data;
        state = 1;
    }
    else if(state==1 && data==0xFF)	//目标地址0xFF
    {
        USART2_RX_BUF[1] = data;
        state = 2;
    }
    else if(state == 2)				//功能码
    {
        if(data == 0x51 || data == 0x34)
        {
            USART2_RX_BUF[2] = data;
            state = 3;
        }
        else state = 0;
    }
    else if(state == 3)				//数据长度
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
    else if(state == 4)				//接收数据
    {
        USART2_RX_BUF[cnt++] = data;
        if(cnt == len + 4)
            state = 5;
    }
    else if(state == 5)				//处理数据
    {
        state = 0;
        Processing_Usart2_Data();	//处理接收到的数据
        return 1;
    }
    else    state = 0;				//一帧接收结束

    return 0;
}

/*
 * 函数名：Processing_Usart3_Data
 * 功  能：处理串口3接收的光流数据，得出所需要的数据
 * 参  数：无
 * 返回值：无
 */
void Processing_Usart2_Data(void)
{
    uint8_t sumcheck = 0,addcheck = 0;
    int Height_t;

    for(uint8_t i=0;i<(USART2_RX_BUF[3] + 4); i++)	//数据累加进行和校验
    {
        sumcheck += USART2_RX_BUF[i];
        addcheck += sumcheck;
    }
    if(sumcheck == USART2_RX_BUF[USART2_RX_BUF[3] + 4] && addcheck == USART2_RX_BUF[USART2_RX_BUF[3] + 5])
    {
        if(USART2_RX_BUF[3] == 0x0F)			//判断是否为位移数据
        {
            DX_SPE = (float)*(short *)&USART2_RX_BUF[6];
            DY_SPE = (float)*(short *)&USART2_RX_BUF[8];
            DX_FIX = (float)*(short *)&USART2_RX_BUF[10];
            DY_FIX = (float)*(short *)&USART2_RX_BUF[12];
            DX_DIS = (float)*(short *)&USART2_RX_BUF[14];
            DY_DIS = (float)*(short *)&USART2_RX_BUF[16];
            DX_LONG += DX_FIX*0.006f;	//计算X方向位移
            DY_LONG += DY_FIX*0.006f;	//计算Y方向位移
            DX_LONG_2 += DX_SPE*0.005f;	//计算X方向位移
            DY_LONG_2 += DY_SPE*0.05f;	//计算Y方向位移
        }
        else if(USART2_RX_BUF[3] == 0x07)
        {
            //高度数据解算
            Height_t = (int)*(int *)&USART2_RX_BUF[7];
            if(Height_t < 240 && Height_t > 0)	//排除误差数据
                Height = Height_t;
            //printf("Height = %d\r\n",Height);
        }
    }
    else;
//        printf("数据解算失败。");
}



