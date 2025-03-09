#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#define PI 3.1415926



typedef struct{
   	float kp, ki, kd; //三个系数
    float kp2, ki2, kd2; //三个系数
    float error, lastError; //误差、上次误差
    float error2, lastError2; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float integral2, maxIntegral2; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
    float output2,maxOutput2; //第二输出
} PID;

//用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Init2(PID *pid, float p, float i, float d, float maxI, float maxOut,float p2, float i2, float d2, float maxI2, float maxOut2);
void PID_Calc(PID *pid, float reference, float feedback);
void PID_Calc2(PID *pid, float angle, float thrust,float ax,float az);