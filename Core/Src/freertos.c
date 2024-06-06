/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "JY901.h"
#include "retarget.h"
#include "tim.h"
#include "usart.h"
#include "PID.h"
#include "ws2812b.h"
#include "beep.h"
#include "oled.h"
#include "optical_flow.h"
#include "move.h"
//#include "arm_math.h"
//#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*************************** 声明全局变量 ***************************/

/************* 高度相关 ***************/
float throttle = 0;    //油门值
float Height_real = 0;  //实际的高度值
float Height_old = 0;  //上次的高度值
float Height_speed = 0;  //高度速度
float Height_expect = 120;  //期望高度
float Height_JY901 = 0;  //由JY901B得到的高度数据（气压计） 未使用
float Height_array[5];   //高度缓冲数组，用于最小二乘法计算

/************* 位置相关 ***************/
float roll = 0, pitch = 0,/* yaw = 0, */Yaw_T265 = 0;  //姿态信息，飞行器实际使用的yaw来自T265
float gyrox = 0, gyroy = 0, gyroz = 0;  //姿态（角速度）信息
float x_expect=0,y_expect=0,yaw_expect=0;  //姿态期望
uint8_t openmv_buf[3] = {0};  //OpenMV数据缓冲，协议为目标位置的坐标（x，y）整型转字符型ASCII码值，第一位帧头
float displacement_x=0,displacement_y=0,displacement_speed_x=0,displacement_speed_y=0;  //水平位置信息
float fire_position_x=0,fire_position_y=0;  //火焰位置信息

/************ 全局标志位 ***************/
int timer = 0;  //全局定时标志位，时长约为1s
uint8_t start_flag = 0;  //正在做题的序号
uint8_t start_temp = 1;  //序号缓冲变量
uint8_t Height_flag = 0;  //定高标志位
uint8_t MOTOR_EN = 0;     //电机使能标志位
float test = 0;

/*******************************END********************************/

/*************************** 各结构图声明 ***************************/

//角度外环
PIDPosition_improve PID_roll; PIDPosition_improve PID_pitch; PIDPosition PID_yaw;
//角度内环
Pid_inc Roll_Velocity; Pid_inc Pitch_Velocity; Pid_inc Yaw_Velocity;
//高度内外环
PIDPosition PID_Height_speed; PIDPosition PID_Height;
//位置外环
PIDPosition_improve PID_location_x; PIDPosition_improve PID_location_y;
PIDPosition_improve velocity_location_x; PIDPosition_improve velocity_location_y;

/*******************************END********************************/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Default_TaskHandle;
osThreadId Sensor_TaskHandle;
osThreadId PID_TaskHandle;
osThreadId KEY_TaskHandle;
osThreadId User_TaaskHandle;
osThreadId VOFA_TaskHandle;
osThreadId POWER_TaskHandle;
osSemaphoreId upstateHandle;
osSemaphoreId downstateHandle;
osSemaphoreId MOTOR_ENHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefault_Task(void const * argument);
void StartSensor_Task(void const * argument);
void StartPID_Task(void const * argument);
void StartKEY_Task(void const * argument);
void StartUser_Task(void const * argument);
void StartVOFA_Task(void const * argument);
void StartPOWER_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void) {
    /* vApplicationMallocFailedHook() will only be called if
    configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
    function that will get called if a call to pvPortMalloc() fails.
    pvPortMalloc() is called internally by the kernel whenever a task, queue,
    timer or semaphore is created. It is also called by various parts of the
    demo application. If heap_1.c or heap_2.c are used, then the size of the
    heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
    FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
    to query the size of free heap space that remains (although it does not
    provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of upstate */
  osSemaphoreDef(upstate);
  upstateHandle = osSemaphoreCreate(osSemaphore(upstate), 1);

  /* definition and creation of downstate */
  osSemaphoreDef(downstate);
  downstateHandle = osSemaphoreCreate(osSemaphore(downstate), 1);

  /* definition and creation of MOTOR_EN */
  osSemaphoreDef(MOTOR_EN);
  MOTOR_ENHandle = osSemaphoreCreate(osSemaphore(MOTOR_EN), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Default_Task */
  osThreadDef(Default_Task, StartDefault_Task, osPriorityIdle, 0, 128);
  Default_TaskHandle = osThreadCreate(osThread(Default_Task), NULL);

  /* definition and creation of Sensor_Task */
  osThreadDef(Sensor_Task, StartSensor_Task, osPriorityRealtime, 0, 512);
  Sensor_TaskHandle = osThreadCreate(osThread(Sensor_Task), NULL);

  /* definition and creation of PID_Task */
  osThreadDef(PID_Task, StartPID_Task, osPriorityRealtime, 0, 1280);
  PID_TaskHandle = osThreadCreate(osThread(PID_Task), NULL);

  /* definition and creation of KEY_Task */
  osThreadDef(KEY_Task, StartKEY_Task, osPriorityAboveNormal, 0, 128);
  KEY_TaskHandle = osThreadCreate(osThread(KEY_Task), NULL);

  /* definition and creation of User_Taask */
  osThreadDef(User_Taask, StartUser_Task, osPriorityHigh, 0, 512);
  User_TaaskHandle = osThreadCreate(osThread(User_Taask), NULL);

  /* definition and creation of VOFA_Task */
  osThreadDef(VOFA_Task, StartVOFA_Task, osPriorityAboveNormal, 0, 512);
  VOFA_TaskHandle = osThreadCreate(osThread(VOFA_Task), NULL);

  /* definition and creation of POWER_Task */
  osThreadDef(POWER_Task, StartPOWER_Task, osPriorityLow, 0, 128);
  POWER_TaskHandle = osThreadCreate(osThread(POWER_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefault_Task */
/**
  * @brief  Function implementing the Default_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefault_Task */
void StartDefault_Task(void const * argument)
{
  /* USER CODE BEGIN StartDefault_Task */
    /* Infinite loop */
    for (;;) {
        osDelay(10000);
    }
  /* USER CODE END StartDefault_Task */
}

/* USER CODE BEGIN Header_StartSensor_Task */
/**
* @brief Function implementing the Sensor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor_Task */
void StartSensor_Task(void const * argument)
{
  /* USER CODE BEGIN StartSensor_Task */

//    TickType_t xPreviousWakeTime;
//    xPreviousWakeTime = xTaskGetTickCount();
    static float DX_SPE_pre=0,DY_SPE_pre=0;

    /* Infinite loop */
    for (;;) {

        taskENTER_CRITICAL();

//        while (JY901_check()) {
//            osDelay(10);
//        }

        Get_Angle_Data(&roll,&pitch);
        Get_Angular_Velocity(&gyrox, &gyroy, &gyroz);

        optical_flow_receive(optical_flow_buf,34);
        HAL_UART_Receive_DMA(&huart2,optical_flow_buf,34);      //这里是光流和高度数据！！！

        HAL_UART_Receive_DMA(&huart3, T265_buf, 14);      //这里是T265数据！！！

        displacement_speed_x = DX_SPE*0.7f+DX_SPE_pre*0.3f;
        displacement_speed_y = DY_SPE*0.7f+DY_SPE_pre*0.3f;
        DX_SPE_pre = DX_SPE;
        DY_SPE_pre = DY_SPE;

//        HAL_UART_Receive_DMA(&huart3, T265_buf, 14);
        UsartReceiveOneData(&T265_x,&T265_y,&Yaw_T265);

//        UsartReceiveOneData(&T265_x,&T265_y,&Yaw_T265);


        taskEXIT_CRITICAL();  //退出临界区，允许任务调度

        osDelay(10);
//        vTaskDelayUntil(&xPreviousWakeTime, 10);
    }
  /* USER CODE END StartSensor_Task */
}

/* USER CODE BEGIN Header_StartPID_Task */
/**
* @brief Function implementing the PID_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPID_Task */
void StartPID_Task(void const * argument)
{
  /* USER CODE BEGIN StartPID_Task */

    uint8_t Pid_count = 0,locat_count = 0;  //计数单位，内环计算两次外环计算一次，以缓解系统快速计算的压力
    uint8_t Height_count = 0;  //高度计数变量
    static float roll_rad=0,pitch_rad=0;  //将角度转换为弧度
    static float Pitch_Pid_out = 0, Roll_Pid_out = 0, Yaw_Pid_out = 0;    //姿态外环PID输出
    static float Pitch_Velocity_out = 0, Roll_Velocity_out = 0, Yaw_Velocity_out = 0;    //姿态内环PID计算结果
//    static float Height_Pid_out = 0;  //高度PID输出 （未使用）
    static float Height_old_speed = 0;
//    static float velocity_x_pre = 0,velocity_y_pre = 0;
    static float Displacement_x_out = 0,Displacement_y_out = 0;
    static float velocity_x_out = 0,velocity_y_out = 0;

    float bbb;  //图像的截距。没有用到，只需要斜率

//    static float x_pre=0,y_pre=0,x_speed_pre=0,y_speed_pre=0;

//    uint32_t previousTimestamp = 0;  // 记录上一次时间戳的变量
//    uint32_t currentTimestamp = 0;   // 记录当前时间戳的变量
//    uint32_t dt = 0;                 // 时间间隔，单位毫秒

    PID_Position_improve_Init(&PID_roll, 1.5f, 0, 0.6f, 100, 100, 100, 10, 0.5f);
    PID_Position_improve_Init(&PID_pitch, 1.5f, 0, 0.6f, 100, 100, 100, 10, 0.5f);
    PID_Position_Init(&PID_yaw, 1.0f, 0, 0, 30, 30, 10);
    PID_Set_Pid_Inc(&Roll_Velocity, 1.4f, 0.065f, 1.0f);
    PID_Set_Pid_Inc(&Pitch_Velocity, 1.4f, 0.065f, 1.0f);
//    PID_Set_Pid_Inc(&Yaw_Velocity, 16.9f, 0.025f, 0.2f);
    PID_Set_Pid_Inc(&Yaw_Velocity, 7.8f, 0.04f, 0);
//    PID_Set_Pid_Inc(&Yaw_Velocity, 0, 0, 0);
    arm_pid_init_f32(&Roll_Velocity, 1);
    arm_pid_init_f32(&Pitch_Velocity, 1);
    arm_pid_init_f32(&Yaw_Velocity, 1);


    //高度环PID结构体赋值
    PID_Position_Init(&PID_Height, 0.5f, 0.05f, 2.0f, 20, 50, 20);
    PID_Position_Init(&PID_Height_speed, 1.0f, 0.001f, 32.0f,20,20,36);

    //位置环PID结构图赋值
    PID_Position_improve_Init(&PID_location_x, 2.6f, 0, 0.5f, 20, 20, 20, 5, 0.1f);
    PID_Position_improve_Init(&PID_location_y, 2.6f, 0, 0.5f, 20, 20, 20, 5, 0.1f);

    //位置速度环PID结构图赋值
    PID_Position_improve_Init(&velocity_location_x, -0.3f, -0.05f, -0.1f, 30, 15, 10, 20, 0.01f);
    PID_Position_improve_Init(&velocity_location_y, -0.3f, -0.05f, -0.1f, 30, 15, 10, 20, 0.01f);

//    PID_Reset_Pid_Inc(&Roll_Velocity);PID_Reset_Pid_Inc(&Pitch_Velocity);PID_Reset_Pid_Inc(&Yaw_Velocity);

//    TickType_t xPreviousWakeTime;
//    // 初始化 xPreviousWakeTime
//    xPreviousWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {

        taskENTER_CRITICAL();  // 进入临界区，屏蔽部分可屏蔽中断

        Pid_count++;
        locat_count++;
        if (Pid_count == 2)Pid_count = 0;
        if (locat_count == 6)Pid_count = 0;
        test = locat_count;

        roll_rad  = (float)cos(roll*3.14159/180);
        pitch_rad = (float)cos(pitch*3.14159/180);

        Height_real = ((float)Height)*roll_rad*pitch_rad;	// 高度校正，斜边转直角边
//        Height_real = Height_real_pre*0.5f + Height_real*0.5f;
//        Height_real_pre = Height_real;

//最小二乘法得高度速度
        if(Height_count == 5)
        {
            Height_count = 0;
            lsqe(X_axis,Height_array,5,&Height_speed,&bbb);  //最小二乘法计算斜率（高度速度）
        }

        Height_array[Height_count] = Height_real;
        Height_count++;

//传统高度速度测量
//        if(Height_count == 2)
//        {
//            Height_count = 0;
//            Height_speed = (float)(Height_real-Height_old);
//            Height_old = Height_real;
//        }

//        Height_array[Height_count] = Height_real;
//        Height_count++;

        Height_speed = (float)(Height_old_speed*0.9+Height_speed*0.1);  //平滑滤波
        Height_old_speed = Height_speed;  //更新数据


        switch(Height_flag)  //无人机的高度状态全部在这里控制。后续如需更改无人机高度方面的动作，只需更改Height_flag的值
        {
            case 1: //初始，快速升速状态。此时飞机不会离地
                if(throttle>600){Height_flag=2;}
                else		throttle+=2;
                break;
            case 2: //接近临界起飞点时，缓慢加速。为了精确获得平衡油门值
                if(Height_real>=40){throttle_balance = throttle;Height_flag=4;}
                else		throttle+=0.5f;
                break;
            case 3: //以平衡油门缓慢上升
                if(Height_real>70){Height_flag=4;}
                else 		throttle=throttle_balance;
                break;
            case 4: //定高
//                throttle = KeepHeight_Remake(4.8f,19.5f,Height_real,Height_speed,Height_expect);
                throttle = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,Height_expect,Height_speed);
//                throttle = throttle_balance + PID_Position_calculate(&PID_Height,Height_expect,Height_real);
                break;
            case 5: //降落
                if(Height_real<18){MOTOR_EN = 0;}
                else if(Height_real<32){throttle=throttle_balance-36;}
//                else    throttle = KeepHeight_Remake(5.0f,13.5f,Height_real,Height_speed,25);
                else throttle = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,28,Height_speed);
                break;
            case 6: //降高度，不落地
//                if(Height_real>80){throttle=throttle_balance-20;}
//                else    throttle = KeepHeight_Remake(4.6f,13.5f,Height_real,Height_speed,40);
//                else throttle = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,60,Height_speed);
                throttle = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,60,Height_speed);
                break;
            case 7: //恢复高度
//                if(Height_real<55){throttle=throttle_balance+20;}
//                else    {throttle = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,Height_expect,Height_speed);
//                            Height_flag = 4;}
                Height_flag = 4;
                break;
            default: MOTOR_EN = 0;break;
        }

        test = KeepHeight(&PID_Height,&PID_Height_speed,Height_real,Height_expect,Height_speed);
//        test = PID_Position_calculate(&PID_Height,Height_expect,Height_real);//如果存在?度差，则会输出速度
//            test = Height_speed;

//        displacement_x = Position_filtering(T265_x,.&x_pre);
//        displacement_y = Position_filtering(T265_y,&y_pre);
//        displacement_speed_x = Position_filtering(T265x_speed,&x_speed_pre);
//        displacement_speed_y = Position_filtering(T265y_speed,&y_speed_pre);

//        if(T265_x < 60 && T265_x > 1 && T265_y < 1 && T265_y > -60)
//        {
//            displacement_x = T265_x*1.25f;
//            displacement_y = T265_y*1.25f;
//        }

//        displacement_x = DX_DIS;
//        displacement_y = DY_DIS;

        //达到临界起飞油门才计算 600
        if(throttle >= 600) {

            if (Pid_count == 1) {  //外环计算
                Displacement_x_out = PID_Position_separate_calculate(&PID_location_x, x_expect, T265_x);
                Displacement_y_out = PID_Position_separate_calculate(&PID_location_y, y_expect, T265_y);

                velocity_x_out = PID_Position_separate_calculate(&velocity_location_x, Displacement_x_out, displacement_speed_x);
                velocity_y_out = PID_Position_separate_calculate(&velocity_location_y, Displacement_y_out, displacement_speed_y);

                Roll_Pid_out = PID_Position_separate_calculate(&PID_roll, velocity_x_out, roll);
                Pitch_Pid_out = PID_Position_separate_calculate(&PID_pitch, velocity_y_out, pitch);
                Yaw_Pid_out = PID_Yaw_Angel_Velocity_Local(&PID_yaw, yaw_expect, Yaw_T265);

            }

            //内环计算
            Roll_Velocity_out = PID_Currency_Inc_Cacl(&Roll_Velocity, Roll_Pid_out, gyrox);
            Pitch_Velocity_out = PID_Currency_Inc_Cacl(&Pitch_Velocity, Pitch_Pid_out, gyroy);
            Yaw_Velocity_out = PID_Currency_Inc_Cacl(&Yaw_Velocity, Yaw_Pid_out, gyroz);  //PID_Yaw_Angel_Velocity_Cacl
        }

        //角度保护
        MPU_Angel_Protection(roll, &MOTOR_EN);
        MPU_Angel_Protection(pitch, &MOTOR_EN);

        if (MOTOR_EN == 1) {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                                  Time_Limit_Pwm((int) (1250 + throttle + Attitude_Result(1, 1, 1))));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                                  Time_Limit_Pwm((int) (1250 + throttle + Attitude_Result(-1, 1, -1))));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                                  Time_Limit_Pwm((int) (1250 + throttle + Attitude_Result(1, -1, -1))));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
                                  Time_Limit_Pwm((int) (1250 + throttle + Attitude_Result(-1, -1, 1))));
        } else {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1250);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1250);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1250);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1250);
        }

        taskEXIT_CRITICAL();  // 退出临界区，允许任务调度


         if(MOTOR_EN == 0)  //对结构体和结果等进行复位
         {
             throttle = 0;
             Height_flag = 0;
             PID_Reset_Pid_improve(&PID_roll);
             PID_Reset_Pid_improve(&PID_pitch);
             PID_Reset_Pid_Local(&PID_yaw);
             Pitch_Velocity_out = 0;
             Roll_Velocity_out = 0;
             Yaw_Velocity_out = 0;
//             Height_Pid_out = 0;
             Displacement_x_out = 0;
             Displacement_y_out = 0;
             arm_pid_init_f32(&Roll_Velocity, 1);
             arm_pid_init_f32(&Pitch_Velocity, 1);
             arm_pid_init_f32(&Yaw_Velocity, 1);
             PID_Reset_Pid_Local(&PID_Height);
             PID_Reset_Pid_Local(&PID_Height_speed);
             start_flag = 0;
//             Height_real_pre = 0;
         }

        osDelay(10);
//        vTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(10));
    }
  /* USER CODE END StartPID_Task */
}

/* USER CODE BEGIN Header_StartKEY_Task */
/**
* @brief Function implementing the KEY_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKEY_Task */
void StartKEY_Task(void const * argument)
{
  /* USER CODE BEGIN StartKEY_Task */

    int i = 0;
    uint8_t Hatch_temp = 0;  //控制货舱舱门
    /* Infinite loop */
    for (;;) {

//        HAL_UART_Receive(&huart1, &USRAT_flag, 1, 50);  //接收无线串口，如起飞指令等信息

//        beep_on();
        if (key_flag == 1 || USRAT_flag == 'a')
        {
            if (i == 1) {
                RGB_Light_Green();
                beep_on();
            } else if (i == 2) {
                RGB_Light_Blue();
                beep_on();
                yaw_expect = Yaw_T265;
            }
            else if (i == 3) {
                beep_on();
                RGB_Light_Red();
                key_flag = 0;
                USRAT_flag = 0;
                MOTOR_EN = 1;
                Height_flag = 1;
                start_flag = start_temp;
            }
            i++;
        }
        else if(key_flag == 2 || USRAT_flag == 'd')
        {
            USRAT_flag = 0;
            Height_flag = 0;
            MOTOR_EN = 0;
            throttle = 0;
            RGB_Light_Green();
            beep_on();beep_on();beep_on();
            key_flag = 0;
        }
        else if(key_flag == 3)  //选择做基础题还是发挥题，1为基础
        {
            key_flag = 0;
            start_temp += 1;
            if(start_temp == 3)
                start_temp = 1;
        }
        else if(key_flag == 4)  //控制舱门开关
        {
            key_flag = 0;
            Hatch_temp += 1;
            if(Hatch_temp == 2)
                Hatch_temp = 0;

            if(Hatch_temp == 1)
                Hatch_on();
            else
                Hatch_off();
        }
//        else if(USRAT_flag == 'H')
//        {
//            USRAT_flag = 0;
//            Height_expect = 40;
//        }
//        else if(USRAT_flag == 'J')
//        {
//            USRAT_flag = 0;
//            Height_expect = 150;
//        }
//        else if(USRAT_flag == 'K')  //此段程序是由无线串口控制停机
//        {
//            USRAT_flag = 0;
//            Height_flag = 5;
//        }
//        else if(USRAT_flag == 'V')
//        {
//            USRAT_flag = 0;
//            x_expect = 100;
//        }
//        else if(USRAT_flag == 'B')
//        {
//            USRAT_flag = 0;
//            x_expect = 0;
//        }
        else {
            i = 0;
        }

        timer++; //全局计时器
        if(timer == 10000) timer = 0;


        osDelay(1000);
//        vTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(1000));
    }
  /* USER CODE END StartKEY_Task */
}

/* USER CODE BEGIN Header_StartUser_Task */
/**
* @brief Function implementing the User_Taask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUser_Task */
void StartUser_Task(void const * argument)
{
  /* USER CODE BEGIN StartUser_Task */
    static int Current_time = 0;  //当前时间记录。非实时，timer才是实时的
    static uint8_t state = 0;     //飞机跑图时的各种状态
    static uint8_t throttle_balance_flag = 1;  //用于补偿平衡油门的信号
    static uint8_t state_temp = 0;             //缓冲状态
    static uint8_t openmv_temp = 0;            //临时变量，用于保证只读取OpenMV一次数据
    /* Infinite loop */
    for (;;) {
//        HAL_UART_Receive_DMA(&huart3, T265_buf, 14);

        if(start_flag == 0)  //准备阶段，屏幕上显示相关信息，出现“OK”字样且下方数据小幅度波动时代表T265就绪，可以启动飞行
        {
            OLED_ShowString(0,0,"Height:",16);
//            if((int)Yaw_T265 < 0)
//            {
//                OLED_ShowString(48,0,"--",16);
//                OLED_ShowNum(80, 0, (int)-Yaw_T265, 3, 16);
//            }
//            else
//            {
//                OLED_ShowString(48,0,"++",16);
//                OLED_ShowNum(80, 0, (int)Yaw_T265, 3, 16);
//            }
            OLED_ShowNum(80, 0, (int)Height_real, 3, 16);
            OLED_ShowString(0,2,"T265:",16);
            OLED_ShowString(0,4,"date:",16);
//            OLED_ShowNum(112,2,(int)point1,2,16);
//            OLED_ShowNum(112,4,(int)point2,2,16);
            if(T265_x != 0 && T265_x < 0.2f && T265_x > -0.2f)
                OLED_ShowString(96,2,"OK",16);
            else
                OLED_ShowString(96,2,"NO",16);

            if(T265_x >= 0)
                OLED_ShowNum(96,4,(int)(T265_x*1000),4,16);
            else
                OLED_ShowNum(96,4,(int)(-T265_x*1000),4,16);

            OLED_ShowNum(112,2,(int)(start_temp),1,16);

            OLED_ShowChinese(32,6,1); //航
            OLED_ShowChinese(48,6,2); //行
            OLED_ShowChinese(64,6,3); //顺
            OLED_ShowChinese(80,6,4); //利
        }


        if(start_flag == 1 || start_flag == 2)  //正在做第一个题、第二个题 主体为共用部分
        {
            switch(state)
            {
                case 0 :
                    if(Height_real > 80 && Height_real < 120)  //已经起飞
                        {
                            state = 1;
                            Current_time = timer;
                        }break;
                case 1 :
//                    if(timer > (Current_time+12) && throttle_balance_flag == 1)
//                    {
//                        throttle_balance += 5;
//                        throttle_balance_flag = 0;
//                    }
                    if(timer > (Current_time+8)/* && Height_real > 60 && Height_real < 100*/)  //检测是否稳定悬停
                    {
//                        if(Yaw_T265 < yaw_expect + 3 && Yaw_T265 > yaw_expect - 3)  //正式跑图
//                        {
//                            position_move(1,&x_expect,&y_expect);  //去1点
//                            state = 2;
//                        }
                        state = 21;
                    }break;
                case 2 :  //2023国赛_优化后的代码
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                            Current_time = timer;
                            position_move(2,&x_expect,&y_expect);  //去2点 转折点
                            state = 3;
                    }break;
                case 3 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        if(throttle_balance_flag == 1)
                        {
                            throttle_balance_flag = 2;
                            throttle_balance += 8;
                        }
                        position_move(3,&x_expect,&y_expect);  //去3点
                        state = 4;
                    }break;
                case 4 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(4,&x_expect,&y_expect);  //去4点 转折点
                        state = 5;
                    }break;
                case 5 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        if(throttle_balance_flag == 2)
                        {
                            throttle_balance_flag = 3;
                            throttle_balance += 5;
                        }
                        position_move(5,&x_expect,&y_expect);  //去5点
                        state = 6;
                    }break;
                case 6 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(6,&x_expect,&y_expect);  //去6点
                        state = 7;
                    }break;
                case 7 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(7,&x_expect,&y_expect);  //去7点
                        state = 8;
                    }break;
                case 8 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(8,&x_expect,&y_expect);  //去8点
                        state = 9;
                    }break;
                case 9 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        if(throttle_balance_flag == 3)
                        {
                            throttle_balance_flag = 4;
                            throttle_balance += 8;
                        }
                        position_move(9,&x_expect,&y_expect);  //去9点
                        state = 10;
                    }break;
                case 10 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(10,&x_expect,&y_expect);  //去10点
                        state = 11;
                    }break;
                case 11 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(11,&x_expect,&y_expect);  //去11点
                        state = 12;
                    }break;
                case 12 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(12,&x_expect,&y_expect);  //去12点
                        state = 13;
                    }break;
                case 13 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        if(throttle_balance_flag == 4)
                        {
                            throttle_balance_flag = 5;
                            throttle_balance += 10;
                        }
                        position_move(13,&x_expect,&y_expect);  //去13点
                        state = 14;
                    }break;
                case 14 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        position_move(14,&x_expect,&y_expect);  //去14点
                        state = 15;
                    }break;
                case 15 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Current_time = timer;
                        if(throttle_balance_flag == 5)
                        {
                            throttle_balance_flag = 0;
                            throttle_balance += 5;
                        }
                        position_move(0,&x_expect,&y_expect);  //回到0点
                        state = 16;
                    }break;
                case 16 :  //我的毕设演示-开始
//                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
//                    {
//                        Height_flag = 5;
//                        start_flag = 0;
//                        state = 17;
//                    }
                    position_move(1,&x_expect,&y_expect);  //去1点
//                    throttle_balance += 5;
                    state = 17;
                case 17 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Height_flag = 6;
                        Current_time = timer;
//                        start_flag = 0;
                        state = 18;
                    }break;
                case 18 :
                    if(timer > (Current_time+5))
                    {
                        Height_flag = 7;
                        Current_time = timer;
                        Hatch_on();
                        beep_on();beep_on();beep_on();
                        state = 19;
                    }break;
                case 19 :
                    if(timer > (Current_time+8))
                    {
//                        Height_flag = 7;
                        position_move(0,&x_expect,&y_expect);  //回到0点
                        state = 20;
                    }break;
                case 20 :  //我的毕设演示-结束
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        Height_flag = 5;
//                        Current_time = timer;
//                        start_flag = 0;
                        state = 21;
                    }break;
                case 21 :  //峰哥的毕设演示-开始
                    position_move(1,&x_expect,&y_expect);  //去1点
                    Current_time = timer;
                    state = 22;
                    break;
                case 22 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        if(timer > (Current_time+8))
                        {
                            position_move(2,&x_expect,&y_expect);  //去2点
                            Current_time = timer;
                            state = 23;
                        }
                    }
                    break;
                case 23 :
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        if(throttle_balance_flag == 1)
                        {
                            throttle_balance_flag = 0;
                            throttle_balance += 6;
                        }
                        if (timer > (Current_time + 8)) {
                            position_move(1, &x_expect, &y_expect);  //去1点
                            Current_time = timer;
                            state = 24;
                        }
                    }break;
                case 24 :  //峰哥的毕设演示-结束
                    if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                    {
                        if (timer > (Current_time + 8))
                        {
                            Height_flag = 5;
                            state = 25;
                        }
                    }break;



                default: break;
            }


            //下面这段代码是2023年的国赛巡航代码 由于频繁使用if..else..语句，已优化为上面的switch形式
            //可以将这段注释掉的代码折叠
/*
            if(state == 0 && Height_real > 170 && Height_real < 190)
            {
                state = 1;
                Current_time = timer;
            }
            if(state == 1 && timer > (Current_time+4) && Height_real > 160 && Height_real < 190 && Yaw_T265 < yaw_expect + 2 && Yaw_T265 > yaw_expect - 2)
            {
//                if(T265_x < 3 && T265_x > -3 && T265_y < 3 && T265_y > -3)
                    state = 2;
//                else
//                    Current_time = timer - 3;
                //Current_time = timer;
            }  //上面为启动（稳定悬停）的判断

            if(state == 2 && Yaw_T265 < yaw_expect + 3 && Yaw_T265 > yaw_expect - 3)  //正式跑图
            {
                position_move(1,&x_expect,&y_expect);  //去1点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 3;
                }
            }
            else if(state == 3 && timer > Current_time)  // timer > Current_time+2
            {
                position_move(2,&x_expect,&y_expect);  //去2点 转折点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    if(throttle_balance_flag == 1)
                    {
                        throttle_balance_flag = 2;
                        throttle_balance += 8;
                    }
                    state = 4;
                }
            }
            else if(state == 4 && timer > Current_time)
            {
                position_move(3,&x_expect,&y_expect);  //去3点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 5;
                }
            }
            else if(state == 5 && timer > Current_time)
            {
                position_move(4,&x_expect,&y_expect);  //去4点 转折点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    if(throttle_balance_flag == 2)
                    {
                        throttle_balance_flag = 3;
                        throttle_balance += 5;
                    }
                    state = 6;
                }
            }
            else if(state == 6 && timer > Current_time)
            {
                position_move(5,&x_expect,&y_expect);  //去5点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 7;
                }
            }
            else if(state == 7 && timer > Current_time)
            {
                position_move(6,&x_expect,&y_expect);  //去6点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 8;
                }
            }
            else if(state == 8 && timer > Current_time)
            {
                position_move(7,&x_expect,&y_expect);  //去7点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 9;
                }
            }
            else if(state == 9 && timer > Current_time)
            {
                position_move(8,&x_expect,&y_expect);  //去8点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    if(throttle_balance_flag == 3)
                    {
                        throttle_balance_flag = 4;
                        throttle_balance += 8;
                    }
                    state = 10;
                }
            }
            else if(state == 10 && timer > Current_time)
            {
                position_move(9,&x_expect,&y_expect);  //去9点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 11;
                }
            }
            else if(state == 11 && timer > Current_time)
            {
                position_move(10,&x_expect,&y_expect);  //去10点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 12;
                }
            }
            else if(state == 12 && timer > Current_time)
            {
                position_move(11,&x_expect,&y_expect);  //去11点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 13;
                }
            }
            else if(state == 13 && timer > Current_time)
            {
                position_move(12,&x_expect,&y_expect);  //去12点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 14;
                    if(throttle_balance_flag == 4)
                    {
                        throttle_balance_flag = 5;
                        throttle_balance += 10;
                    }
                }
            }
            else if(state == 14 && timer > Current_time)
            {
                position_move(13,&x_expect,&y_expect);  //去13点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 15;
                }
            }
            else if(state == 15 && timer > Current_time)
            {
                position_move(13,&x_expect,&y_expect);  //去13点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    if(throttle_balance_flag == 5)
                    {
                        throttle_balance_flag = 0;
                        throttle_balance += 5;
                    }
                    state = 16;
                }
            }
            else if(state == 16 && timer > Current_time)
            {
                position_move(14,&x_expect,&y_expect);  //去14点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 17;
                }
            }
            else if(state == 17 && timer > Current_time)
            {
                position_move(0,&x_expect,&y_expect);  //回到0点
                if(T265_x > x_expect-1 && T265_x < x_expect+1 && T265_y > y_expect-1 && T265_y < y_expect+1)
                {
                    Current_time = timer;
                    state = 18;
                }
            }
            else if(state == 18 && timer > Current_time)
            {
                Height_flag = 5;
                start_flag = 0;
            } //如果只做第一个题，程序在上面就终止

            if(start_flag == 2)  //正在做第二个题
            {
                if(openmv_temp == 0)
                {
                    HAL_UART_Receive(&huart6, openmv_buf, 3, 50);
                    state_temp = state;
                }

                if(openmv_buf[0] == 0x80 && openmv_temp == 0)
                {
                    state = 20;
                    openmv_temp = 1;
                    x_expect = T265_x;
                    y_expect = T265_y;
//                    beep_on();beep_on();beep_on();
                    Light_on();
                    Openmv_Data_change(openmv_buf[2],-openmv_buf[1],T265_x,T265_y,&fire_position_x,&fire_position_y);
                }

                if(timer > Current_time+2 && state == 20) {Height_flag = 6; Current_time = timer;state = 21;}  //下降高度
                if(timer > Current_time+5 && state == 21) {Hatch_on(); Current_time = timer;state = 22;Light_off();}  //开舱门
                if(timer > Current_time+3 && state == 22) {state = state_temp;Hatch_off(); Height_flag = 7;}  //恢复高度
            }*/
        }

        osDelay(200);
    }
  /* USER CODE END StartUser_Task */
}

/* USER CODE BEGIN Header_StartVOFA_Task */
/**
* @brief Function implementing the VOFA_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVOFA_Task */
void StartVOFA_Task(void const * argument)
{
  /* USER CODE BEGIN StartVOFA_Task */
//    TickType_t xPreviousWakeTime;
//    xPreviousWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {
//    xQueueSend(Queue_SignalHandle,&res_buf,0);

//    xQueueReceive(Queue_PIDHandle,&roll,0);
//    xQueueReceive(Queue_PIDHandle,&pitch,0);
//    xQueueReceive(Queue_PIDHandle,&Yaw_T265,0);

//        Vofa_Input((float) throttle_balance, 0);
//        Vofa_Input((float) throttle, 1);
//        Vofa_Input((float) Height_real, 2);//DX_DIS T265_x
//        Vofa_Input((float) Height_speed, 3);//DY_DIS T265_y
//        Vofa_Send();

        Laser_dotting_base(); //激光笔闪烁
        osDelay(200);
//        vTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(1000));
    }
  /* USER CODE END StartVOFA_Task */
}

/* USER CODE BEGIN Header_StartPOWER_Task */
/**
* @brief Function implementing the POWER_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPOWER_Task */
void StartPOWER_Task(void const * argument)
{
  /* USER CODE BEGIN StartPOWER_Task */
//    TickType_t xPreviousWakeTime;
//    xPreviousWakeTime = xTaskGetTickCount();

    /* Infinite loop */
    for (;;) {
        osDelay(500000);
//        vTaskDelayUntil(&xPreviousWakeTime, pdMS_TO_TICKS(1000));
    }
  /* USER CODE END StartPOWER_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
