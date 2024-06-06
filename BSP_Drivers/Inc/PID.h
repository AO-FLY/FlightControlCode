//
// Created by Haoyee_Yin on 2023/7/15.
//

#ifndef __PID_H
#define __PID_H

#include "arm_math.h"

extern float throttle_balance;
extern float X_axis[50];

typedef struct {
    float Kp;                 //比例系数
    float Ki;                 //积分系数
    float Kd;                 //微分系数
    float integral;           //积分值
    float prev_error;         //上一次的误差
    float integral_limit;     //积分限幅值
    float derivative_limit;   //微分限幅值
    float output_limit;       //输出限幅值
} PIDPosition;  //位置式PID控制器结构体

typedef struct {
    float Kp;                 //比例系数
    float Ki;                 //积分系数
    float Kd;                 //微分系数
    float integral;           //积分值
    float integral_trigger;   //积分触发值
    float error_dead;         //误差死区值
    float prev_error;         //上一次的误差
    float prev_derivative;    //上一次的微分
    float integral_limit;     //积分限幅值
    float derivative_limit;   //微分限幅值
    float output_limit;       //输出限幅值
} PIDPosition_improve;  //改良位置式PID控制器结构体

typedef struct {
    float Kp;                 //比例系数
    float Ki;                 //积分系数
    float Kd;                 //微分系数
    float prev_output;        //上一次的输出
    float prev_error;         //上一次的误差
    float Last_error;         //上上次的误差
} PIDIncrement;  //增量式PID控制器结构体

typedef arm_pid_instance_f32 Pid_inc;	//增量式pid结构体声明

/*
 * 封装arm_dsp库中PID复位函数
 * 函数原型：void arm_pid_init_f32()
 * 封 装 后：void PID_Reset_Pid_Inc()
 * 参    数：S --> Pid_inc结构体指针	state --> 初始化标志
 */
#define PID_Reset_Pid_Inc(S) arm_pid_init_f32((Pid_inc *)S, 1)

/*
 * 封装arm_dsp库中PID计算函数
 * 函数原型：float arm_pid_f32()
 * 封 装 后：float PID_Cacl_Inc()
 * 参    数：S --> Pid_inc结构体指针	err --> 偏差值
 * 备    注：此函数为增量式PID计算公式
 */
#define PID_Cacl_Inc(S, err) arm_pid_f32((Pid_inc *)S, (float) err)

void PID_Position_Init(PIDPosition* pid, float Kp, float Ki, float Kd, float integral_limit, float derivative_limit, float output_limit);
void PID_Increment_Init(PIDIncrement* pid, float Kp, float Ki, float Kd);
float PID_Position_calculate(PIDPosition* pid, float setpoint, float measured_value);
float PID_Yaw_Angel_Velocity_Local(PIDPosition* pid, float setpoint, float measured_value);
float PID_Position_Height_calculate(PIDPosition* pid, float setpoint, float measured_value);
float KeepHeight(PIDPosition* pid_Height1, PIDPosition* pid_Height_speed1, float measurementHeight, float Height_expect1, float Height_Speed1);//定高特殊PID函数
float KeepHeight_Remake(float stiffness, float damping, float mesurementHeight, float Height_speed_t, float Height_Expet);//定高改良
float PID_Increment_calculate(PIDIncrement* pid,float setpoint, float measured_value);
int Time_Limit_Pwm( int pwm_t );
float constrainf(float input, float min, float max);

float PID_Position_separate_calculate(PIDPosition_improve* pid, float setpoint, float measured_value); //改良PID
void PID_Reset_Pid_improve(PIDPosition_improve *pid);
void PID_Position_improve_Init(PIDPosition_improve* pid, float Kp, float Ki, float Kd, float integral_limit, float derivative_limit, float output_limit, float integral_trigger, float error_dead);

float PID_Currency_Inc_Cacl(Pid_inc *S,float expect,float now);
float PID_Yaw_Angel_Velocity_Cacl(Pid_inc *S,float expect,float now);
void PID_Reset_Pid_Local(PIDPosition *pid);
void PID_Set_Pid_Inc(Pid_inc *S, float Kp_t, float Ki_t, float Kd_t);
void PID_Reset_Pid_Increment(PIDIncrement *pid);

void lsqe(float *x,float *y,int n ,float *k,float *b);

#define Attitude_Result(P,R,Y)	constrainf((float)(P*Pitch_Velocity_out+R*Roll_Velocity_out+Y*Yaw_Velocity_out),-80,80)

#endif //__PID_H
