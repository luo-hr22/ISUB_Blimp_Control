#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.1415926
#include "PID.h"


//用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut){
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}

void PID_Init2(PID *pid, float p, float i, float d, float maxI, float maxOut,float p2, float i2, float d2, float maxI2, float maxOut2){
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    pid->kp2 = p2;
    pid->ki2 = i2;
    pid->kd2 = d2;
    pid->maxIntegral2 = maxI2;
    pid->maxOutput2 = maxOut2;
}

//进行一次pid计算（角度）
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float reference, float feedback){

 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = reference - feedback; //计算新error

    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;

    //计算比例
    float pout = pid->error * pid->kp;

    //计算积分
    pid->integral += pid->error * pid->ki;

    //积分限幅
    if (pid->integral > pid->maxIntegral)
    {
        pid->integral = pid->maxIntegral;
    }
    else if (pid->integral < - pid->maxIntegral)
    {
        pid->integral = - pid->maxIntegral;
    }

    //计算输出
    pid->output = pout + dout + pid->integral;

    //输出限幅
    if (pid->output > pid->maxOutput)
    {
        pid->output =   pid->maxOutput;
    }
    else if (pid->output < - pid->maxOutput)
    {
        pid->output = - pid->maxOutput;
    }
}

void PID_Calc2(PID *pid, float angle, float thrust,float ax,float az){
    float value;
 	//更新数据
    pid->lastError = pid->error;
    pid->lastError2 = pid->error2;
    pid->error = ax; //水平
    pid->error2 = az; //竖直

    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
    float dout2 = (pid->error2 - pid->lastError2) * pid->kd2;

    //计算比例
    float pout = pid->error * pid->kp;
    float pout2 = pid->error2 * pid->kp2;

    //计算积分
    pid->integral += pid->error * pid->ki;
    pid->integral2 += pid->error2 * pid->ki2;

    //积分限幅
    if (pid->integral > pid->maxIntegral)
    {
        pid->integral = pid->maxIntegral;
    }
    else if (pid->integral < - pid->maxIntegral)
    {
        pid->integral = - pid->maxIntegral;
    }

    if (pid->integral2 > pid->maxIntegral2)
    {
        pid->integral2 = pid->maxIntegral2;
    }
    else if (pid->integral2 < - pid->maxIntegral2)
    {
        pid->integral2 = - pid->maxIntegral2;
    }

    //计算输出
    float sum = pout + dout + pid->integral;
    float sum2 = pout2 + dout2 + pid->integral2;
    pid->output = sqrt(pow(thrust * cos(angle) - sum, 2) + pow(thrust*sin(angle)-sum2,2));
    value= 180.0 / 3.1415926 * atan((thrust * sin(angle) - sum) / (thrust * cos(angle) - sum2));
    if ((thrust * sin(angle) - sum)>=0)
    {
        if (value>=0) pid->output2=value;
        else pid->output2=180+value;
    }
    else
    {
        if (value>=0) pid->output2=value+180;
        else pid->output2=360+value;
    }
    //输出限幅
    if (pid->output > pid->maxOutput)
    {
        pid->output =   pid->maxOutput;
    }
    else if (pid->output < - pid->maxOutput)
    {
        pid->output = - pid->maxOutput;
    }
    if (pid->output2 > pid->maxOutput2)
    {
        pid->output2 =   pid->maxOutput2;
    }
    else if (pid->output2 < - pid->maxOutput2)
    {
        pid->output2 = - pid->maxOutput2;
    }
}




/*
int main(){
    PID mypid = {0};
    PID yourpid = {0}; 
    float Acc[3], Gyro[3], Angle[3];
	
	
    //...这里有些其他初始化代码

    PID_Init(&mypid, 0.2, 0.01, 0.1, 2000, 2000);
    PID_Init2(&yourpid, 0.2, 0.01, 0.1, 2000, 2000, 0.2, 0.01, 0.1, 2000, 2000);

    while(1)
    {   getparam(Acc, Gyro, Angle);
		
		
		
        float feedbackValue = Angle[2];
        float targetValue = 0;
        float axvalue =Acc_x/16384.0 ;
        float azvalue = Acc_z/16384.0;

        PID_Calc(&mypid, targetValue, feedbackValue);
        PID_Calc2(&yourpid, Angle[2], 0.5*(Speed_L+Speed_R), Acc[0], Acc[1]);

        设定执行器输出大小(mypid.output);
        设定执行器输出大小(yourpid.output);

        delay(10);
    }

}
*/