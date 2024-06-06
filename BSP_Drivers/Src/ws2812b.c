//
// Created by Haoyee_yin on 2023/7/18.
//

#include "ws2812b.h"

//uint32_t WS2812_Data[WS2812_Num] = {0x00ff00};

#ifdef WS2812_Software
void WS2812_Send_Byte(uint8_t Dat);
/**
 * @brief 发送0码
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:01:10
 */
inline void WS2812_Code_0(void)
{
    HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_SET);
    for (int i = 0; i < 6; i++)
        __nop();
    HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < 11; i++)
        __nop();
}
/**
 * @brief 发送1码
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:01:10
 */
inline void WS2812_Code_1(void)
{
    HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_SET);
    for (int i = 0; i < 12; i++)
        __nop();
    HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < 4; i++)
        __nop();
}
/**
 * @brief 发送1Byte数据
 * @param Dat:数据
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:01:10
 */
inline void WS2812_Send_Byte(uint8_t Dat)
{
    for (int i = 0; i < 8; i++)
    {
        if (Dat & 0x80)
        {
            WS2812_Code_1();
        }
        else
        {
            WS2812_Code_0();
        }
        Dat <<= 1;
    }
}
/**
 * @brief 发送复位码
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:01:10
 */
void WS2812_Code_Reast(void)
{
    HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}
/**
 * @brief 解码颜色
 * @param Color:颜色
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:02:07
 */
void WS2812_Color_Decode(uint32_t Color)
{
    WS2812_Send_Byte((Color & 0x00ff00) >> 8);
    WS2812_Send_Byte((Color & 0xff0000) >> 16);
    WS2812_Send_Byte((Color & 0x0000ff) >> 0);
}
/**
 * @brief 发送数据给灯
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:02:28
 */
void WS2812_Send(void)
{
    for (int i = 0; i < 2; i++)
    {
        WS2812_Code_Reast();
        for (int i = 0; i < WS2812_Num; i++)
        {
            WS2812_Color_Decode(WS2812_Data[i]);
        }
        HAL_GPIO_WritePin(WS2812_GPIO_Group, WS2812_GPIO_Pin, GPIO_PIN_SET);
    }
}
#endif

#ifdef WS2812_Hardware
uint32_t WS2812_SendBuf0[25] = {0};   //发送缓冲区0
uint32_t WS2812_SendBuf1[25] = {0};   //发送缓冲区1
const uint32_t WS2812_Rst[240] = {0}; //复位码缓冲区
uint32_t WS2812_En = 0;               //发送使能
/**
 * @brief 将uint32转为发送的数据
 * @param Data:颜色数据
 * @param Ret:解码后的数据(PWM占空比)
 * @return
 * @author HZ12138
 * @date 2022-10-03 18:03:17
 */
void WS2812_uint32ToData(uint32_t Data, uint32_t *Ret)
{
    uint32_t zj = Data;
    uint8_t *p = (uint8_t *)&zj;
    uint8_t R = 0, G = 0, B = 0;
    B = *(p);     // B
    G = *(p + 1); // G
    R = *(p + 2); // R
    zj = (G << 16) | (R << 8) | B;
    for (int i = 0; i < 24; i++)
    {
        if (zj & (1 << 23))
            Ret[i] = WS2812_Code_1;
        else
            Ret[i] = WS2812_Code_0;
        zj <<= 1;
    }
    Ret[24] = 0;
}
/**
 * @brief 发送函数(DMA中断调用)
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:04:50
 */
void WS2812_Send(void)
{
    static uint32_t j = 0;
    static uint32_t ins = 0;
    if (WS2812_En == 1)
    {
        if (j == WS2812_Num)
        {
            j = 0;
            HAL_TIM_PWM_Stop_DMA(&WS2812_TIM, WS2812_TIM_Channel);
            WS2812_En = 0;
            return;
        }
        j++;
        if (ins == 0)
        {
            HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_Channel, WS2812_SendBuf0, 25);
            WS2812_uint32ToData(WS2812_Data[j], WS2812_SendBuf1);
            ins = 1;
        }
        else
        {
            HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_Channel, WS2812_SendBuf1, 25);
            WS2812_uint32ToData(WS2812_Data[j], WS2812_SendBuf0);
            ins = 0;
        }
    }
}
/**
 * @brief 开始发送颜色数据
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:05:13
 */
void WS2812_Start(void)
{
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_Channel, (uint32_t *)WS2812_Rst, 240);
    WS2812_uint32ToData(WS2812_Data[0], WS2812_SendBuf0);
    WS2812_En = 1;
}
/**
 * @brief 发送复位码
 * @param 无
 * @return 无
 * @author HZ12138
 * @date 2022-10-03 18:05:33
 */
void WS2812_Code_Reast(void)
{
    HAL_TIM_PWM_Start_DMA(&WS2812_TIM, WS2812_TIM_Channel, (uint32_t *)WS2812_Rst, 240);
    WS2812_En = 0;
}
#endif

//#define T0H() __NOP();__NOP();__NOP();__NOP();
//#define T0L() __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//#define T1H() __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
//#define T1L() __NOP();__NOP();__NOP();__NOP();

// 纳秒延时函数
void delay_ns(uint32_t ns)
{
    TIM2->ARR = ns - 1;       // 设置定时器的自动重载值
    TIM2->EGR = TIM_EGR_UG;   // 更新生成事件，将新的 ARR 值加载到计数器
    TIM2->SR &= ~TIM_SR_UIF;  // 清除溢出标志
    TIM2->CR1 |= TIM_CR1_CEN; // 启动定时器

    while (!(TIM2->SR & TIM_SR_UIF)) {
        // 等待定时器溢出
    }

    TIM2->CR1 &= ~TIM_CR1_CEN; // 关闭定时器
    TIM2->SR &= ~TIM_SR_UIF;   // 清除溢出标志
}

void delay_04us(void)
{
    // 根据实际情况调整这段代码的执行时间
    for (int i = 0; i < 6; i++) {
        __NOP();
    }
}
void delay_08us(void)
{
    // 根据实际情况调整这段代码的执行时间
    for (int i = 0; i < 20; i++) {
        __NOP();
    }
}

void RGB_On(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
}

void RGB_Off(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void RGB_Start(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void Ws2812_Send(uint8_t data)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        if(data & 0x80)
        {
            RGB_On();
            T1H();
            RGB_Off();
            T1L();
        }
        else
        {
            RGB_On();
            T0H();
            RGB_Off();
            T0L();
        }
        data<<=1;
    }

}

void ws218_Clear(void)
{
    Ws2812_Send(0x00);
    Ws2812_Send(0x00);
    Ws2812_Send(0x00);
    delay_ns(300000);
}

void RGB_Lighting(uint8_t *str)
{
    int i;

    RGB_Start();
    for(i=0;i<3;i++)
    {
        Ws2812_Send(*str);
        str++;
    }
}

//RGB红色
void RGB_Light_Red(void)
{
    uint8_t C_Red[]={0x00,0xff,0x00};		//红色
    RGB_Lighting(C_Red);
}

//RGB绿色
void RGB_Light_Green(void)
{
    uint8_t C_Green[]={0xff,0x00,0x00};	    //蓝色
    RGB_Lighting(C_Green);
}

//RGB蓝色
void RGB_Light_Blue(void)
{
    uint8_t C_Blue[]={0x00,0x00,0xff};		//蓝色
    RGB_Lighting(C_Blue);
}

//RGB紫色
void RGB_Light_Purple(void)
{
    uint8_t C_Purple[]={0x3C,0x8D,0xC4};	//紫色
    RGB_Lighting(C_Purple);
}

//RGB粉色
void RGB_Light_Pink(void)
{
    uint8_t C_Pink[]={0x00,0xff,0xff};		//粉色
    RGB_Lighting(C_Pink);
}

//RGB黄色
void RGB_Light_Yellow(void)
{
    uint8_t C_Yellow[]={0xff,0xff,0x00};	//黄色
    RGB_Lighting(C_Yellow);
}

//RGB橙色
void RGB_Light_Orange(void)
{
    uint8_t C_Orange[]={0x7f,0xff,0x24};	//橙色
    RGB_Lighting(C_Orange);
}


