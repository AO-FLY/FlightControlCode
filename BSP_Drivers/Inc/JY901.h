/*
 * JY901头文件
 * 
 * 包含各内部寄存器地址，官网或淘宝客服上可以找到
 * 
 * Created on: 2023年7月11日
 * Author: 奉曦先生
 * 
 */


#ifndef __JY901_H_
#define __JY901_H_

#include "i2c.h"


#define JY901B_I2C_ADDR 0x50  //JY901 IIC地址
#define JY901B_I2C_Handle hi2c1  //JY901 使用的IIC外设 这里是IIC1


/*
 * SCL——PB6
 * SDA——PB7
 */
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}
//#define IIC1_SCL_PIN	GPIO_PIN_6
//#define IIC1_SDA_PIN	GPIO_PIN_7
//#define IIC1_GPIO_PORT	GPIOB
#define IIC1_SCL(a)		HAL_GPIO_WritePin(IIC1_GPIO_PORT, IIC1_SCL_PIN, a)
#define IIC1_SDA(a)		HAL_GPIO_WritePin(IIC1_GPIO_PORT, IIC1_SDA_PIN, a)
#define IIC1_Read_SDA()	HAL_GPIO_ReadPin(IIC1_GPIO_PORT, IIC1_SDA_PIN)



#define GPIO_PORT_I2C	GPIOB			     // GPIO端口
#define I2C_SCL_PIN		GPIO_PIN_6			 // 连接到SCL时钟线的GPIO
#define I2C_SDA_PIN		GPIO_PIN_7			 // 连接到SDA数据线的GPIO
/* 定义读写SCL和SDA的宏 */
#define I2C_SCL_1()  GPIO_PORT_I2C->BSRR = I2C_SCL_PIN											// SCL = 1
#define I2C_SCL_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SCL_PIN << 16U  		// SCL = 0
#define I2C_SDA_1()  GPIO_PORT_I2C->BSRR = I2C_SDA_PIN   										// SDA = 1
#define I2C_SDA_0()  GPIO_PORT_I2C->BSRR = (uint32_t)I2C_SDA_PIN << 16U  		// SDA = 0
#define I2C_SDA_READ()  (GPIO_PORT_I2C->IDR & I2C_SDA_PIN)										// 读SDA口线状态
#define I2C_SCL_READ()  (GPIO_PORT_I2C->IDR & I2C_SCL_PIN)									// 读SCL口线状态





#define SAVE 		0x00	//保存当前配置
#define CALSW 		0x01	//校准
#define RSW 		0x02	//回传数据内容
#define RRATE		0x03	//回传数据速率
#define BAUD 		0x04	//串口波特率

#define AXOFFSET	0x05	//X轴加速度零偏
#define AYOFFSET	0x06	//Y轴加速度零偏
#define AZOFFSET	0x07	//Z轴加速度零偏
#define GXOFFSET	0x08	//X轴角速度零偏
#define GYOFFSET	0x09	//Y轴角速度零偏
#define GZOFFSET	0x0a	//Z轴角速度零偏
#define HXOFFSET	0x0b	//X轴磁场零偏
#define HYOFFSET	0x0c	//Y轴磁场零偏
#define HZOFFSET	0x0d	//Z轴磁场零偏

#define D0MODE		0x0e	//D0模式
#define D1MODE		0x0f	//D1模式
#define D2MODE		0x10	//D2模式
#define D3MODE		0x11	//D3模式
#define D0PWMH		0x12	//D0PWM高电平宽度
#define D1PWMH		0x13	//D1PWM高电平宽度
#define D2PWMH		0x14	//D2PWM高电平宽度
#define D3PWMH		0x15	//D3PWM高电平宽度
#define D0PWMT		0x16	//D0D0PWM周期
#define D1PWMT		0x17	//D1D0PWM周期
#define D2PWMT		0x18	//D2D0PWM周期
#define D3PWMT		0x19	//D3D0PWM周期

#define IICADDR		0x1a	//IIC地址
#define LEDOFF 		0x1b	//关闭LED指示灯

#define YYMM		0x30	//年、月
#define DDHH		0x31	//日、时
#define MMSS		0x32	//分、秒
#define MS			0x33	//毫秒
#define AX			0x34	//X轴加速度
#define AY			0x35	//Y轴加速度
#define AZ			0x36	//Z轴加速度
#define GX			0x37	//X轴角速度
#define GY			0x38	//Y轴角速度
#define GZ			0x39	//Z轴角速度
#define HX			0x3a	//X轴磁场
#define HY			0x3b	//Y轴磁场	
#define HZ			0x3c	//Z轴磁场
#define Roll		0x3d	//X轴角度
#define Pitch		0x3e	//Y轴角度
#define Yaw			0x3f	//Z轴角度
#define TEMP		0x40	//模块温度
#define D0Status	0x41	//D0端口状态
#define D1Status	0x42	//D0端口状态
#define D2Status	0x43	//D0端口状态
#define D3Status	0x44	//D0端口状态
#define PressureL	0x45	//气压低字节
#define PressureH	0x46	//气压高字节
#define HeightL		0x47	//高度低字节
#define HeightH		0x48	//高度高字节
#define LonL		0x49	//经度低字节
#define LonH		0x4a	//经度高字节
#define LatL		0x4b	//纬度低字节
#define LatH		0x4c	//纬度高字节
#define GPSHeight  	0x4d	//GPS高度
#define GPSYAW     	0x4e	//GPS航向角
#define GPSVL		0x4f	//GPS地速低字节
#define GPSVH		0x50	//GPS地速高字节
#define q0			0x51	//四元素Q0
#define q1			0x52	//四元素Q1
#define q2			0x53	//四元素Q1
#define q3			0x54	//四元素Q1


void IIC1_Start(void);
void IIC1_Stop(void);

//声明.c文件中各项函数
unsigned char JY901_check(void);
//void Get_Angle_Data(float *roll_t,float *pitch_t,float *yaw_t);  //旧的
void Get_Angle_Data(float *roll_t,float *pitch_t); //新的
void Get_Angular_Velocity(float *gyrox_t,float *gyroy_t,float *gyroz_t);  //旧的
//void Get_Angular_Velocity(float *gyrox_t,float *gyroy_t);  //新的
void Get_Acceleration(float *accx_t,float *accy_t,float *accz_t);
void Get_Magnetic_Flied(float *hx_t,float *hy_t,float *hz_t);
void Get_Height_Data(float *Height_JY901);
void Get_Atmospheric_Pressure(float *pressure);
void MPU_Angel_Protection(float angel, uint8_t *state);

#endif
