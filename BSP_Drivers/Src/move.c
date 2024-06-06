//
// Created by Haoyee_yin on 2023/8/1.
//
#include "move.h"
#include "tim.h"


//�������� 2022��ʡ��
//int Point_array[13][2]={{0,0},{50,275},{200,125},{275,200},{350,-25},
//                        {350,275},{275,50},{125,50},{125,200},{50,125},
//                        {200,-25},{200,275},{350,275}};

//�������� 2023���������λ�����ף�
//int Point_array[12][2]={{40,40},{440,40},{440,360},{360,360},
//                        {360,120},{280,120},{280,360},{200,360},
//                        {200,120},{120,120},{120,360},{40,360}};

//�������� 2023����� (��λ������)
//int Point_array[13][2]={{4,4},{44,4},{44,36},{36,36},
//                        {36,12},{28,12},{28,36},{20,36},
//                        {20,12},{12,12},{12,36},{4,36},{3,3}};

//�������� 2023����� (��λ������)
//int Point_array[14][2]={{3,3},{24,4},{44,4},{44,18},
//                        {44,36},{36,36},{36,12},{28,12},
//                        {28,36},{20,36},{20,12},{12,12},
//                        {12,36},{4,36}};

////�������� 2023����� (��λ������)
//int Point_array[15][2]={{3,3},{4,18},{4,36},{12,36},{12,12},
//                        {20,12},{20,36},{28,36},{28,12},
//                        {36,12},{36,36},{44,36},{44,18},
//                        {44,4},{24,4}
//                        };
//
////ÿ���������������
//int Area_array[30][2]=
//        {{4,4},{12,4},{20,4},{28,4},{36,4},{44,4},
//         {44,12},{44,20},{44,28},{44,36},{36,36},{36,28},
//         {36,20},{36,12},{28,12},{28,20},{28,28},
//         {28,36},{20,36},{20,28},{20,20},{20,12},
//         {12,12},{12,20},{12,28},{12,36},{4,36},
//         {4,28},{4,20},{4,12}};
//

//�������� ����ѭ����λ (��λ������)
int Point_array[15][2]={{0,0},{0,10},{14,10},{12,36},{12,12},
                        {20,12},{20,36},{28,36},{28,12},
                        {36,12},{36,36},{44,36},{44,18},
                        {44,4},{24,4}
                        };

/*
 * ���ڸ���λ�û�����ֵ
 * num����2022��ʡ��/2023�����ͼ�ϵĵ�λ���
 */
void position_move(uint8_t num,float* x_expect_t,float* y_expect_t)
{
    *y_expect_t = -(float)(Point_array[num][0] - Point_array[0][0]);
    *x_expect_t = (float)(Point_array[num][1] - Point_array[0][1]);
}

/*
 * �ɻ��·�����ʴ��Ļ�������
 * ����Դ�㣬������һ��
 */
void Laser_dotting_base(void)
{
//    HAL_GPIO_WritePin(GPIOD, PD0_Pin, 1); // ��������
//    HAL_Delay(100);
//    HAL_GPIO_WritePin(GPIOD, PD0_Pin, 0);
//    HAL_Delay(100);

    HAL_GPIO_TogglePin(GPIOD,PD0_Pin);  //��ת��ƽ
}

//������
void Hatch_on()
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 2500);
}

//�ز���
void Hatch_off()
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 500);
}

//����ʾ��
void Light_on()
{
    HAL_GPIO_WritePin(GPIOD, Light_Pin, 0);
}

//�ؾ�ʾ��
void Light_off()
{
    HAL_GPIO_WritePin(GPIOD, Light_Pin, 1);
}


//openmv���ݽ���
void Openmv_Data_change(uint8_t openmv_rebuf1,uint8_t openmv_rebuf2,float dis_x,float dis_y,float* fire_x,float* fire_y)
{
    if(openmv_rebuf1 > 127)
        *fire_x = (float)(openmv_rebuf1 - 256)*10 + dis_x;
    else
        *fire_x = (float)openmv_rebuf1*10 + dis_x;

    if(openmv_rebuf2 > 127)
        *fire_y = (float)(openmv_rebuf2 - 256)*10 + dis_y;
    else
        *fire_y = (float)openmv_rebuf2*10 + dis_y;
}


