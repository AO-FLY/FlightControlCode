/*
 * JY901.c
 *
 * JY901 以下数据提取与解算函数。
 *
 * Created on: 2023年7月7日
 * Author: 奉曦先生
 * 
 */

#ifdef HardI2C

#include "JY901.h"


//以下函数使用前请确保IIC已完成初始化

//JY901检查函数 检查设备是否能够正常通信
unsigned char JY901_check()
{
    uint8_t JY901_status;
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,0x1a, I2C_MEMADD_SIZE_8BIT,&JY901_status,1,100);

    if(JY901_status == 0x50) return 0;
    else return 1;
}

//获取传感器角度数据
void Get_Angle_Data(float *roll_t,float *pitch_t,float *yaw_t)
{
    uint8_t angle_data[6];
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,Roll, I2C_MEMADD_SIZE_8BIT,angle_data,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,Pitch, I2C_MEMADD_SIZE_8BIT,angle_data+2,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,Yaw, I2C_MEMADD_SIZE_8BIT,angle_data+4,2,100);

    *roll_t  = ((float)((short)((short)angle_data[1]<<8)|angle_data[0]))/32768*180;
	*pitch_t = ((float)((short)((short)angle_data[3]<<8)|angle_data[2]))/32768*180;
	*yaw_t   = ((float)((short)((short)angle_data[5]<<8)|angle_data[4]))/32768*180;
}

//获取传感器角速度数据
void Get_Angular_Velocity(float *gyrox_t,float *gyroy_t,float *gyroz_t)
{
    uint8_t angular_velocity[6];
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,GX, I2C_MEMADD_SIZE_8BIT,angular_velocity,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,GY, I2C_MEMADD_SIZE_8BIT,angular_velocity+2,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,GZ, I2C_MEMADD_SIZE_8BIT,angular_velocity+4,2,100);

    *gyrox_t = ((float)((short)((short)angular_velocity[1]<<8)|angular_velocity[0]))/32768*2000;
	*gyroy_t = ((float)((short)((short)angular_velocity[3]<<8)|angular_velocity[2]))/32768*2000;
	*gyroz_t = ((float)((short)((short)angular_velocity[5]<<8)|angular_velocity[4]))/32768*2000;
}

//获取传感器加速度数据
void Get_Acceleration(float *accx_t,float *accy_t,float *accz_t)
{
    uint8_t acceleration[6];
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AX, I2C_MEMADD_SIZE_8BIT,acceleration,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AY, I2C_MEMADD_SIZE_8BIT,acceleration+2,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AZ, I2C_MEMADD_SIZE_8BIT,acceleration+4,2,100);

    *accx_t = ((float)((short)((short)acceleration[1]<<8)|acceleration[0]))/32768*16;
	*accy_t = ((float)((short)((short)acceleration[3]<<8)|acceleration[2]))/32768*16;
	*accz_t = ((float)((short)((short)acceleration[5]<<8)|acceleration[4]))/32768*16;
}

//获取磁场数据
void Get_Magnetic_Flied(float *hx_t,float *hy_t,float *hz_t)
{
	uint8_t magnetic_data[6];

    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HX, I2C_MEMADD_SIZE_8BIT,magnetic_data,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HY, I2C_MEMADD_SIZE_8BIT,magnetic_data+2,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HZ, I2C_MEMADD_SIZE_8BIT,magnetic_data+4,2,100);

	*hx_t = ((float)((short)((short)magnetic_data[1]<<8)|magnetic_data[0]));
	*hy_t = ((float)((short)((short)magnetic_data[3]<<8)|magnetic_data[2]));
	*hz_t = ((float)((short)((short)magnetic_data[5]<<8)|magnetic_data[4]));
}

//获取气压计高度数据
void Get_Height_Data(float *Height_JY901)
{
	uint8_t Height_Data[4];

    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HeightL, I2C_MEMADD_SIZE_8BIT,Height_Data,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HeightH, I2C_MEMADD_SIZE_8BIT,Height_Data+2,2,100);

    *Height_JY901 = ((float)((long)((long)((long)((long)Height_Data[3]<<24)|Height_Data[2]<<16)|Height_Data[1]<<8)|Height_Data[0]));
}

//获取气压数据
void Get_Atmospheric_Pressure(float *pressure)
{
	uint8_t Pressure[4];

    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,PressureL, I2C_MEMADD_SIZE_8BIT,Pressure,2,100);
    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,PressureH, I2C_MEMADD_SIZE_8BIT,Pressure+2,2,100);

    *pressure = ((float)((long)((long)((long)((long)Pressure[3]<<24)|Pressure[2]<<16)|Pressure[1]<<8)|Pressure[0]));
}


//角度保护 超45°停机
void MPU_Angel_Protection(float angel, uint8_t *state)
{
	if(angel > 45 || angel < -45)
	    *state = 0;
}

#endif

#define SoftI2C

#ifdef SoftI2C

#include "JY901.h"
#include "usart.h"
#include "tim.h"

void IIC_Delay(uint16_t us)
{
    uint16_t differ=0xffff-us-5;
    HAL_TIM_Base_Start(&htim7);
            __HAL_TIM_SetCounter(&htim7,differ);
    while(differ < 0xffff-5)
    {
        differ = __HAL_TIM_GetCounter(&htim7);
    }
    HAL_TIM_Base_Stop(&htim7);
}

/*    _____
 *SDA      \_____________
 *    __________
 *SCL           \________
 */
void IIC1_Start(void) { // 开始,SCL为高电平的时候SDA产生一个下降沿信号
//    SDA_OUT();//sda线输出
//    IIC1_SDA(GPIO_PIN_SET); //先拉高SDA再拉高SCL，防止出现错误信号
//    IIC1_SCL(GPIO_PIN_SET);
//    IIC_Delay(2);
//    IIC1_SDA(GPIO_PIN_RESET);
//    IIC_Delay(2);
//    IIC1_SCL(GPIO_PIN_RESET); //钳住I2C总线，准备发送或接收数据

    I2C_SDA_1();
    I2C_SCL_1();
    IIC_Delay(2);
    I2C_SDA_0();
    IIC_Delay(2);
    I2C_SCL_0();
//    IIC_Delay(2);
}

/*               _______
 *SDA __________/
 *          ____________
 *SCL _____/
 */
void IIC1_Stop(void) { // I2C 停止,SCL为高电平的时候SDA产生一个上升沿信号
//    SDA_OUT();//sda线输出
//    IIC1_SCL(GPIO_PIN_RESET);
//    IIC1_SDA(GPIO_PIN_RESET); //先拉低SDA再拉高SCL，防止出现错误信号
//    IIC_Delay(2);
//    IIC1_SCL(GPIO_PIN_SET);
//    IIC1_SDA(GPIO_PIN_SET);
//    IIC_Delay(2);


    I2C_SCL_0();
    I2C_SDA_0();
    IIC_Delay(2);
    I2C_SCL_1();
    I2C_SDA_1();
    IIC_Delay(2);
}

/**
 * @brief I2C 等待响应
 * @retval none
 * @author Mr.W
 * @date 2020-10-12
 */
uint8_t IIC1_Wait_Ack(void)
{
//    uint8_t ucErrTime = 0;
//    SDA_IN();      //SDA设置为输入
//    /* IIC_SDA=1;-------------------------------------------------------------*/
//    IIC1_SDA(GPIO_PIN_SET);
//    IIC_Delay(2);
//    /* IIC_SCL=1;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_SET);
//    IIC_Delay(2);
//    while ( IIC1_Read_SDA() == GPIO_PIN_SET )
//    {
//        ucErrTime++;
//        if (ucErrTime > 250)
//        {
//            IIC1_Stop();
//            return 1;
//        }
//    }
//    /* IIC_SCL=0; 时钟输出0---------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);
//    return 0;

    uint8_t re;
    I2C_SDA_1();	/* CPU释放SDA总线 */
//	i2c_Delay();
    I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
    IIC_Delay(2);
    if (I2C_SDA_READ())	/* CPU读取SDA口线状态 */
    {
        re = 1;
    }
    else
    {
        re = 0;
    }
    I2C_SCL_0();
    IIC_Delay(2);
    return re;
}

/*           ____
 *SCL ______/    \______
 *    ____         _____
 *SDA     \_______/
 */
void IIC1_Ack(void) { // 产生ACK应答
//    /* IIC_SCL=0;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);
////    SDA_OUT();
//    /* IIC_SDA=0;-------------------------------------------------------------*/
//    IIC1_SDA(GPIO_PIN_RESET);
//    IIC_Delay(2);
//    /* IIC_SCL=1;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_SET);
//    IIC_Delay(2);
//    /* IIC_SCL=0;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);

    I2C_SDA_0();	/* CPU驱动SDA = 0 */
    IIC_Delay(2);
    I2C_SCL_1();	/* CPU产生1个时钟 */
    IIC_Delay(2);
    I2C_SCL_0();
    IIC_Delay(2);
    I2C_SDA_1();	/* CPU释放SDA总线 */
}

/*           ____
 *SCL ______/    \______
 *    __________________
 *SDA
 */
void IIC1_NAck(void) { // 不响应
//    /* IIC_SCL=0;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);
////    SDA_OUT();
//    /* IIC_SDA=1;-------------------------------------------------------------*/
//    IIC1_SDA(GPIO_PIN_SET);
//    IIC_Delay(2);
//    /* IIC_SCL=1;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_SET);
//    IIC_Delay(2);
//    /* IIC_SCL=0;-------------------------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);

    I2C_SDA_1();	/* CPU驱动SDA = 0 */
    IIC_Delay(2);
    I2C_SCL_1();	/* CPU产生1个时钟 */
    IIC_Delay(2);
    I2C_SCL_0();
    IIC_Delay(2);
    I2C_SDA_1();	/* CPU释放SDA总线 */
}

void IIC1_Send_Byte(uint8_t data) { // 发送一个字节的数据
//    uint8_t t;
////    SDA_OUT();
//    /* IIC_SCL=0; 拉低时钟开始数据传输------------------------------------------*/
//    IIC1_SCL(GPIO_PIN_RESET);
//    for (t = 0; t < 8; t++)
//    {
//        //IIC_SDA = ( txd & 0x80 ) >> 7;
//        if ((data & 0x80) >> 7)
//        {
//            IIC1_SDA(GPIO_PIN_SET);
//        }
//        else
//        {
//            IIC1_SDA(GPIO_PIN_RESET);
//        }
//        data<<=1;
//        /* IIC_SCL=1; --------------------------------------------------------*/
//        IIC1_SCL(GPIO_PIN_SET);
//        IIC_Delay(2);
//        /* IIC_SCL=0; 拉低时钟开始数据传输-------------------------------------*/
//        IIC1_SCL(GPIO_PIN_RESET);
//        IIC_Delay(2);
//    }

    uint8_t i;
    // 先发送字节的高位bit7
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            I2C_SDA_1();
        }
        else
        {
            I2C_SDA_0();
        }
        IIC_Delay(2);
        I2C_SCL_1();
        IIC_Delay(2);
        I2C_SCL_0();
        if (i == 7)
        {
            I2C_SDA_1(); 	// 释放总线
        }
        data <<= 1;		// 左移一个bit
        IIC_Delay(2);
    }
}

uint8_t IIC1_Read_Byte(uint8_t ack) { // 读1个字节，ack=1时，发送ACk；ack=0，发送NACK
//    unsigned char i,receive=0;
////    SDA_IN();//SDA设置为输入
//    for(i=0;i<8;i++ )
//    {
//        /* IIC_SCL=0;---------------------------------------------------------*/
//        IIC1_SCL(GPIO_PIN_RESET);
//        IIC_Delay(2);
//        /* IIC_SCL=1;---------------------------------------------------------*/
//        IIC1_SCL(GPIO_PIN_SET);
//        receive<<=1;
//        if (IIC1_Read_SDA() == GPIO_PIN_SET)
//            receive++;
//        IIC_Delay(2);
//    }
//    if (ack)
//        IIC1_Ack();//发送ACK
//    else
//        IIC1_NAck(); //发送NACK
//    return receive;

    uint8_t i;
    uint8_t value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    for (i = 0; i < 8; i++)
    {
        value <<= 1;
        I2C_SCL_1();
        IIC_Delay(2);
        if (I2C_SDA_READ())
        {
            value++;
        }
        I2C_SCL_0();
        IIC_Delay(2);
    }
    if (ack)
        IIC1_Ack();//发送ACK
    else
        IIC1_NAck(); //发送NACK
    return value;
}

uint8_t JY901_Read_Byte(uint8_t addr,uint8_t reg)
{
    uint8_t res;
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC1_Wait_Ack();             //等待应答
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答

    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack();             //等待应答
    res = IIC1_Read_Byte(0);		//读数据,发送nACK
    IIC1_Stop();                 //产生一个停止条件
    return res;
}

uint8_t JY901_Read_Bytes(uint8_t *buf,uint8_t addr,uint8_t reg,uint8_t len)
{
    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC1_Wait_Ack())          //等待应答
    {
        IIC1_Stop();
        return 1;
    }
    IIC1_Send_Byte(reg);         //写寄存器地址
    IIC1_Wait_Ack();             //等待应答

    IIC1_Start();
    IIC1_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC1_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC1_Read_Byte(0);//读数据,发送nACK
        else *buf=IIC1_Read_Byte(1);		//读数据,发送ACK
        len--;
        buf++;
    }
    IIC1_Stop();                 //产生一个停止条件
    return 0;
}

//检查是否与陀螺仪完成通信
//返回值 0正常 1不正常
unsigned char JY901_check()
{
//    uint8_t ack;
//    IIC1_Start();
//    IIC1_Send_Byte((JY901B_I2C_ADDR<<1)|0);
//    ack = IIC1_Wait_Ack();
//    IIC1_Stop();
//    return ack;

    uint8_t ack;
    if (I2C_SDA_READ() && I2C_SCL_READ())
    {
        IIC1_Start();		/* 发送启动信号 */
        /* 发送设备地址+读写控制bit（0 = w， 1 = r) bit7 先传 */
        IIC1_Send_Byte((JY901B_I2C_ADDR << 1) | 0);
        ack = IIC1_Wait_Ack();	/* 检测设备的ACK应答 */
        IIC1_Stop();			/* 发送停止信号 */
        return ack;
    }
    return 1;	/* I2C总线异常 */
}

//获取传感器角度数据
//void Get_Angle_Data(float *roll_t,float *pitch_t,float *yaw_t)
//{
//    uint8_t angle_data[6] = "";
//    if (JY901_Read_Bytes(angle_data, JY901B_I2C_ADDR, Roll, 6) == 0) {
//        *roll_t = ((float) ((short) ((short) angle_data[1] << 8) | angle_data[0])) / 32768.0f * 180.0f;
//        *pitch_t = ((float)((short)((short)angle_data[3]<<8)|angle_data[2]))/32768.0f * 180.0f;
//        *yaw_t = ((float) ((short) ((short) angle_data[5] << 8) | angle_data[4])) / 32768.0f * 180.0f;
//    }
//}


//获取传感器角度数据  新的
void Get_Angle_Data(float *roll_t,float *pitch_t)
{
    uint8_t angle_data[4] = "";
    if (JY901_Read_Bytes(angle_data, JY901B_I2C_ADDR, Roll, 4) == 0) {
        *roll_t = ((float) ((short) ((short) angle_data[1] << 8) | angle_data[0])) / 32768.0f * 180.0f;
        *pitch_t = ((float)((short)((short)angle_data[3]<<8)|angle_data[2]))/32768.0f * 180.0f;
//        *yaw_t = ((float) ((short) ((short) angle_data[5] << 8) | angle_data[4])) / 32768.0f * 180.0f;
    }
}

//获取传感器角速度数据
void Get_Angular_Velocity(float *gyrox_t,float *gyroy_t,float *gyroz_t)
{
    uint8_t angular_velocity[6] = "";
    if (JY901_Read_Bytes(angular_velocity, JY901B_I2C_ADDR, GX, 6) == 0) {
        *gyrox_t = ((float) ((short) ((short) angular_velocity[1] << 8) | angular_velocity[0])) / 32768 * 2000;
        *gyroy_t = ((float)((short)((short)angular_velocity[3]<<8)|angular_velocity[2]))/32768*2000;
        *gyroz_t = ((float) ((short) ((short) angular_velocity[5] << 8) | angular_velocity[4])) / 32768 * 2000;
    }
}


//获取传感器角速度数据  新的  由于T265没有输出yaw速度 这个放弃使用
//void Get_Angular_Velocity(float *gyrox_t,float *gyroy_t)
//{
//    uint8_t angular_velocity[4] = "";
//    if (JY901_Read_Bytes(angular_velocity, JY901B_I2C_ADDR, GX, 4) == 0) {
//        *gyrox_t = ((float) ((short) ((short) angular_velocity[1] << 8) | angular_velocity[0])) / 32768 * 2000;
//        *gyroy_t = ((float)((short)((short)angular_velocity[3]<<8)|angular_velocity[2]))/32768*2000;
////        *gyroz_t = ((float) ((short) ((short) angular_velocity[5] << 8) | angular_velocity[4])) / 32768 * 2000;
//    }
//}

//获取传感器加速度数据
void Get_Acceleration(float *accx_t,float *accy_t,float *accz_t)
{
    uint8_t acceleration[6] = "";
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AX, I2C_MEMADD_SIZE_8BIT,acceleration,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AY, I2C_MEMADD_SIZE_8BIT,acceleration+2,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,AZ, I2C_MEMADD_SIZE_8BIT,acceleration+4,2,1000);

    if (JY901_Read_Bytes(acceleration, JY901B_I2C_ADDR, AX, 6) == 0) {
        *accx_t = ((float) ((short) ((short) acceleration[1] << 8) | acceleration[0])) / 32768 * 16;
        *accy_t = ((float) ((short) ((short) acceleration[3] << 8) | acceleration[2])) / 32768 * 16;
        *accz_t = ((float) ((short) ((short) acceleration[5] << 8) | acceleration[4])) / 32768 * 16;
    }
}

//获取磁场数据
void Get_Magnetic_Flied(float *hx_t,float *hy_t,float *hz_t)
{
    uint8_t magnetic_data[6];

//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HX, I2C_MEMADD_SIZE_8BIT,magnetic_data,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HY, I2C_MEMADD_SIZE_8BIT,magnetic_data+2,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HZ, I2C_MEMADD_SIZE_8BIT,magnetic_data+4,2,1000);

    *hx_t = ((float)((short)((short)magnetic_data[1]<<8)|magnetic_data[0]));
    *hy_t = ((float)((short)((short)magnetic_data[3]<<8)|magnetic_data[2]));
    *hz_t = ((float)((short)((short)magnetic_data[5]<<8)|magnetic_data[4]));
}

//获取气压计高度数据
void Get_Height_Data(float *Height_JY901)
{
    uint8_t Height_Data[4] = "";

//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HeightL, I2C_MEMADD_SIZE_8BIT,Height_Data,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,HeightH, I2C_MEMADD_SIZE_8BIT,Height_Data+2,2,1000);

    if (JY901_Read_Bytes(Height_Data, JY901B_I2C_ADDR, HeightL, 4) == 0) {
        *Height_JY901 = ((float) (
                (long) ((long) ((long) ((long) Height_Data[3] << 24) | Height_Data[2] << 16) | Height_Data[1] << 8) |
                Height_Data[0]));
    }
}

//获取气压数据
void Get_Atmospheric_Pressure(float *pressure)
{
    uint8_t Pressure[4];

//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,PressureL, I2C_MEMADD_SIZE_8BIT,Pressure,2,1000);
//    HAL_I2C_Mem_Read(&JY901B_I2C_Handle,(JY901B_I2C_ADDR<<1)|0x00,PressureH, I2C_MEMADD_SIZE_8BIT,Pressure+2,2,1000);

        *pressure = ((float) ((long) ((long) ((long) ((long) Pressure[3] << 24) | Pressure[2] << 16) | Pressure[1] << 8) | Pressure[0]));
}


//角度保护 超30°停机
void MPU_Angel_Protection(float angel, uint8_t *state)
{
    if(angel > 30 || angel < -30)
    {
        *state = 0;
        key_flag = 2;
    }
}





#endif

