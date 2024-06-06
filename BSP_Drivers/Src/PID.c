//
// Created by Haoyee_Yin on 2023/7/15.
//
// PID算法等计算程序
//   内容有点多，调用需要的函数即可
//

#include "PID.h"
//#include "usart.h"

float throttle_balance = 0;

//float X_axis[50]={0     ,0.005f,0.010f,0.015f,0.020f, 0.025f,0.030f,0.035f,0.040f,0.045f,
//            0.050f,0.055f,0.060f,0.065f,0.070f,0.075f,0.080f,0.085f,0.090f,0.095f,
//            0.100f,0.105f,0.110f,0.115f,0.120f,0.125f,0.130f,0.135f,0.140f,0.145f,
//            0.150f,0.155f,0.160f,0.165f,0.170f,0.175f,0.180f,0.185f,0.190f,0.195f,
//            0.200f,0.205f,0.210f,0.215f,0.220f,0.225f,0.230f,0.235f,0.240f,0.245f,
//};

//float X_axis[50]={0      ,0.010f,0.020f,0.030f,0.040f, 0.050f,0.060f,0.070f,0.080f,0.090f,
//                  0.100f,0.110f,0.120f,0.130f,0.140f,0.150f,0.160f,0.170f,0.180f,0.190f,
//                  0.200f,0.210f,0.220f,0.230f,0.240f,0.250f,0.260f,0.270f,0.280f,0.290f,
//                  0.300f,0.310f,0.320f,0.330f,0.340f,0.350f,0.360f,0.370f,0.380f,0.390f,
//                  0.400f,0.410f,0.420f,0.430f,0.440f,0.450f,0.460f,0.470f,0.480f,0.490f,
//};

float X_axis[50]={0      ,0.020f,0.040f,0.060f,0.080f, 0.100f,0.120f,0.140f,0.160f,0.180f,
                  0.200f,0.220f,0.240f,0.260f,0.280f,0.300f,0.320f,0.340f,0.360f,0.380f,
                  0.400f,0.420f,0.440f,0.460f,0.480f,0.500f,0.520f,0.540f,0.560f,0.580f,
                  0.600f,0.620f,0.640f,0.660f,0.680f,0.700f,0.720f,0.740f,0.760f,0.780f,
                  0.800f,0.820f,0.840f,0.860f,0.880f,0.900f,0.920f,0.940f,0.960f,0.980f,
};

//位置式PID初始化，送入参数和限幅值
void PID_Position_Init(PIDPosition* pid, float Kp, float Ki, float Kd, float integral_limit, float derivative_limit, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->derivative_limit = derivative_limit;
    pid->output_limit = output_limit;
}

//增量式PID初始化，送入参数和限幅值
void PID_Increment_Init(PIDIncrement* pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_output = 0;
    pid->prev_error = 0;
    pid->Last_error = 0;
}

//改良式位置式PID初始化，送入参数和限幅值
void PID_Position_improve_Init(PIDPosition_improve* pid, float Kp, float Ki, float Kd, float integral_limit, float derivative_limit, float output_limit, float integral_trigger, float error_dead)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->integral_limit = integral_limit;
    pid->derivative_limit = derivative_limit;
    pid->output_limit = output_limit;
    pid->integral_trigger = integral_trigger;
    pid->error_dead = error_dead;
}



//位置式PID算法，能够满足系统准确性
float PID_Position_calculate(PIDPosition* pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    float output = 0;

    // Proportional term
    output = pid->Kp * error;

    // Integral term
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    output += pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error);
    if (derivative > pid->derivative_limit) {
        derivative = pid->derivative_limit;
    }
    else if (derivative < -pid->derivative_limit) {
        derivative = -pid->derivative_limit;
    }
    output += pid->Kd * derivative;

    // Output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // Save previous error for next iteration
    pid->prev_error = error;

    return output;
}

//位置式PID算法，能够满足系统准确性，此函数用于yaw角调节
float PID_Position_yaw_calculate(PIDPosition* pid, float error)
{
    float output = 0;

    // Proportional term
    output = pid->Kp * error;

    // Integral term
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    output += pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error);
    if (derivative > pid->derivative_limit) {
        derivative = pid->derivative_limit;
    }
    else if (derivative < -pid->derivative_limit) {
        derivative = -pid->derivative_limit;
    }
    output += pid->Kd * derivative;

    // Output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // Save previous error for next iteration
    pid->prev_error = error;

    return output;
}



//改良位置式PID算法，能够满足系统准确性
float PID_Position_separate_calculate(PIDPosition_improve* pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    float output = 0;

    if(error < pid->error_dead && error > -pid->error_dead)
        error = 0;

    // Proportional term
    output = pid->Kp * error;

    // Integral term
    if(error < pid->integral_trigger && error > -pid->integral_trigger)
        pid->integral += error;
    else
        pid->integral = 0;

    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    output += pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error);
    if (derivative > pid->derivative_limit) {
        derivative = pid->derivative_limit;
    }
    else if (derivative < -pid->derivative_limit) {
        derivative = -pid->derivative_limit;
    }
    output += (float)((pid->Kd) * ((0.5 * derivative)+(0.5 * pid->prev_derivative)));
    pid->prev_derivative = derivative;

    // Output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // Save previous error for next iteration
    pid->prev_error = error;

    return output;
}



//高度位置式PID算法，积分项反向
float PID_Position_Height_calculate(PIDPosition* pid, float setpoint, float measured_value)
{
    float error = setpoint - measured_value;
    float output = 0;

    // Proportional term
//    output = pid->Kp * error;

    // Integral term
    pid->integral += error;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
//    output += pid->Ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error);
    if (derivative > pid->derivative_limit) {
        derivative = pid->derivative_limit;
    }
    else if (derivative < -pid->derivative_limit) {
        derivative = -pid->derivative_limit;
    }
//    output += pid->Kd * derivative;

    output = pid->Kp * error - pid->Ki * pid->integral + pid->Kd * derivative;

    // Output limit
    if (output > pid->output_limit) {
        output = pid->output_limit;
    }
    else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    // Save previous error for next iteration
    pid->prev_error = error;

    return output;
}

//定高特殊PID函数
float KeepHeight(PIDPosition* pid_Height1, PIDPosition* pid_Height_speed1, float measurementHeight, float Height_expect1, float Height_Speed1)
{

    static float result = 0;
    static float resultPre = 0;
    static float outH = 0, outV = 0;//高度度环输出，速度环输出
//    static uint8_t temp = 0;
//
//    if(temp == 5) temp = 0;
//    if(temp == 0)
//    {
//
//    }
    outH = PID_Position_calculate(pid_Height1,Height_expect1,measurementHeight);//如果存在?度差，则会输出速度
    outH = constrainf(outH, -10,10);//输出限幅
    outV = PID_Position_calculate(pid_Height_speed1,outH,Height_Speed1);//达到上升的速度
    outV = constrainf(outV, -36,36);//输出限幅
    if(Height_expect1>=40)
        result = throttle_balance+outV;
    else
        result = throttle_balance+outV-30;
//    result = throttle_balance + outV;
    result = result * 0.25f + resultPre * 0.75f;//输出滤波，限制突变
    resultPre = result;
//    temp ++;
    return result;
}

//定高特殊PID函数_重制版
float KeepHeight_Remake(float stiffness, float damping, float mesurementHeight, float Height_speed_t, float Height_Expet)
{
    float result;
    static float resultPre;
    static float outPre;
    float out;//高度度环输出，速度环输出

    out = stiffness * (Height_Expet - mesurementHeight) - damping * Height_speed_t;
    out = constrainf(out, -300,300);//输出限幅
    out = (float)(out * 0.2 + outPre * 0.8);//输出滤波，限制突变

    result = throttle_balance + out;
//    result = (float)(result * 0.2 + resultPre * 0.8);//输出滤波，限制突变
//    resultPre = result;

    return result;
}


// 增量式PID算法，能够满足系统快速性
float PID_Increment_calculate(PIDIncrement* pid,float setpoint, float measured_value)
{
    // 计算偏差
    float error = setpoint - measured_value;

    // 计算增量式PID控制器的增量
    float delta_p = pid->Kp * (error - pid->prev_error);
    float delta_i = pid->Ki * error;
    float delta_d = pid->Kd * (error - 2*pid->prev_error + pid->Last_error);

    // 计算本次输出
    float output = pid->prev_output + delta_p + delta_i + delta_d;

    // 更新状态变量
    pid->prev_output = output;
    pid->Last_error = pid->prev_error;
    pid->prev_error = error;

    return output;
}

/*
 * 函数名：Time_Limit_Pwm
 * 功  能：限制PWM范围在500~900之间
 * 参  数：
 */
int Time_Limit_Pwm( int pwm_t )
{
    if(pwm_t > 2250)
        pwm_t = 2250;
    else if(pwm_t < 1250)
        pwm_t = 1250;

    return pwm_t;
}

//限幅函数
float constrainf(float input, float min, float max)
{
    if (input < min)
        input = min;
    if (input > max)
        input = max;
    return input;
}

/*
 * 函数名：PID_Currency_Inc_Cacl
 * 功  能：通用增量式PID计算
 * 参  数：*S --> 增量式PID结构体	*now:当前值	*expect:期望值
 */
float PID_Currency_Inc_Cacl(Pid_inc *S,float expect,float now)
{
    float err;
    float out;

    //计算偏差
    err = expect - now;
//    printf("%d  ",(int)err);

    out = PID_Cacl_Inc(S,err);

    return out;
}

/*
 * 函数名：PID_Yaw_Angel_Velocity_Cacl
 * 功  能：姿态环控制-偏航角角度内环计算
 * 参  数：*S --> 角速度增量式PID结构体	*now:当前值	*expect:期望值
 */
float PID_Yaw_Angel_Velocity_Cacl(Pid_inc *S,float expect,float now)
{
    float err;
    float out;

    //计算偏差
    if(now - expect >180 || now - expect < -180)
    {
        if(expect > 0 && now < 0)
            err = (-180 - now) + (expect - 180);
        if(expect < 0 && now > 0)
            err = ( 180 - now) + (expect + 180);
    }
    else
        err = expect - now;

    //PID计算输出
    out = PID_Cacl_Inc(S,err);

    return out;
}

//yaw角专用外环调节函数
float PID_Yaw_Angel_Velocity_Local(PIDPosition* pid, float setpoint, float measured_value)
{
    float err;
    float out;

    //计算偏差
    if(measured_value - setpoint >180 || measured_value - setpoint < -180)
    {
        if(setpoint > 0 && measured_value < 0)
            err = (-180 - measured_value) + (setpoint - 180);
        if(setpoint < 0 && measured_value > 0)
            err = ( 180 - measured_value) + (setpoint + 180);
    }
    else
        err = setpoint - measured_value;

    out = PID_Position_yaw_calculate(pid, err);

    return out;
}

/*
 * 函数名：PID_Reset_Pid_Local()
 * 功  能：复位位置式PID中的积分和误差为0
 * 参  数：*S -> 位置式PID结构体
 */
void PID_Reset_Pid_Local(PIDPosition *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

/*
 * 函数名：PID_Set_Pid_Inc()
 * 功  能：设置增量式PID
 * 参  数：*S -> 增量式PID结构体	Kp_t -> 设置kp的值	Ki_t -> 设置ki的值	Kd_t -> 设置kd的值
 */
void PID_Set_Pid_Inc(Pid_inc *S, float Kp_t, float Ki_t, float Kd_t)
{
    S->Kp = Kp_t;
    S->Ki = Ki_t;
    S->Kd = Kd_t;
}

//改良PID复位
void PID_Reset_Pid_improve(PIDPosition_improve *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_derivative = 0.0f;
}

/*
 * 函数名：PID_Reset_Pid_Local()
 * 功  能：复位位置式PID中的积分和误差为0
 * 参  数：*S -> 位置式PID结构体
 */
void PID_Reset_Pid_Increment(PIDIncrement *pid)
{
    pid->prev_output = 0;
    pid->prev_error = 0;
    pid->Last_error = 0;
}

//最小二乘法求直线斜率
/*
	函数输出：无
	函数功能：求取数组的斜率
	函数输入：X轴基地址，Y轴基地址，数组长度，斜率k指针，直线的b指针
*/
void lsqe(float *x,float *y,int n ,float *k,float *b)
{
    float sumx,sumy,sumx2,sumxy;
    int i=0;
    sumx =0.0f;
    sumy =0.0f;
    sumx2=0.0f;
    sumxy=0.0f;
    for (i=0;i<n;i++)
    {
        sumx = sumx+x[i];
        sumy = sumy+y[i];
        sumx2=(float)(sumx2+pow(x[i],2));
        sumxy=sumxy+(x[i] * y[i]);
    }
    *k=(float)((sumxy-((sumx*sumy)/(float)n))/(sumx2-(pow(sumx,2.0)/(float)n)));
    *b=(sumy-(*k)*sumx)/(float)n;
}


