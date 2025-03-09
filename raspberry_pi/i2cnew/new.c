#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <unistd.h>
#include "digitalPWM.h"
#include "i2c.h"
#include "wit_c_sdk.h"
#include "getparam.h"
#include "PID.h"

//#define会报错，请用变量
int asbst = 0;//0代表树莓派向后
//int pin_r=22;//Right_Motor,BCM18
//int pin_l=25;//Left_Motor,BCM12
int pin_s=26;//Servo,BCM13
//以上四行注意查证
int pin_r1 = 21;
int pin_r2 = 27;
int pin_l1 = 22;
int pin_l2 = 23;//修改

int va, angle;
int speed_r = 0;//Virtual Speed, range:[-1000,1000]
int speed_l = 0;//^^^^^^^^^^^^^
int aver = 0;
int mark = 0;//mark==1---stopped
int r_last, l_last;

void printscreen(void)
{
    printf("Speed_L=%d, Speed_R=%d, Angle=%d\n",speed_l-800,speed_r-800,angle);
}

void servo(int Angle)
{   Angle=Angle*180/3.1415926;
    if (Angle>=180) Angle-=180;//大于180？
    if (asbst == 0) va = ((180.0 - Angle) / 9.0 + 5) * 10;
    else va = (Angle / 9.0 + 5) * 10;
    pwmWrite(pin_s, va);
    //printscreen();
}

void motors(int pin1, int pin2, int v) 
{
    if (v > 0)
    {
        softPwmWrite(pin1, v);
        softPwmWrite(pin2, 0);
    }
    else
    {
        softPwmWrite(pin1, 0);
        softPwmWrite(pin2, -v);
    }
}


/*int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}*///键盘控制


int upd(void)
{
    motors(pin_r1, pin_r2, speed_r);
    motors(pin_l1, pin_l2, speed_l);
    aver = 0.5 * (speed_r + speed_l);
    //printscreen();
}

int ini(void)
{
    if (wiringPiSetup() == -1) 
    {
        printf("wiringPi initialization error\n");
        return -1; // 初始化wiringPi失败，返回错误
    }
    printf("Initializing servo...\n");
    pinMode(pin_s, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);
    pwmSetRange(2000);
    pwmWrite(pin_s, 250);
    printf("Initializing motor...\n");
    softPwmCreate(pin_r1, 0, 100);
    softPwmCreate(pin_r2, 0, 100);
    softPwmCreate(pin_l1, 0, 100);
    softPwmCreate(pin_l2, 0, 100);
    printf("Initialized\n");
    speed_l = 0;
    speed_r = 0;
    angle = asbst * 180;
}

int main(){
    angle = asbst * 180;
    PID mypid = {0};
    PID yourpid = {0}; 
    float Acc[3], Gyro[3], Angle[3];
    float feedbackValue;
    float targetValue;
    float axvalue;
    float azvalue;
    printf("Enter to start\n");
    scanf("");
    ini();
    speed_l = 0;
    speed_r = 0;
    delay(500);
    upd();

    float maxiV = 2000;
    float maxiAngle = 180;
    float maxiI = 1000000000;
    float maxiI2 = 1000000000;
    float k0[] = { 10, 0.01, 0.1 };//kp,ki,kd for mypid
    float k1[] = { 80, 0.01, 0.1 };//kp,ki,kd for yourpid
    float k2[] = { 80, 0.01, 0.1 };//kp2,ki2,kd2 for yourpid


    PID_Init(&mypid, k0[0], k0[1], k0[2], maxiI, maxiV);
    PID_Init2(&yourpid, k1[0], k1[1], k1[2], maxiI, maxiV, k2[0], k2[1], k2[2], maxiI2, maxiAngle);

    while(1)
    {   getparam(Acc, Gyro, Angle);
		
        feedbackValue = Angle[2];
        targetValue = 0;
        //axvalue = Acc[0] /16384.0 ;
        //azvalue = Acc[2] / 16384.0;

        PID_Calc(&mypid, targetValue, feedbackValue);
        PID_Calc2(&yourpid, angle, 0.5*(speed_l+speed_r), Acc[0], Acc[2]-1);
        //yourpidoutput2在大于180度后反转
        if (yourpid.output2>=3.1415926) aver = -yourpid.output;
        else aver = yourpid.output;
        speed_l = aver + mypid.output;
        speed_r = aver - mypid.output;
        angle = yourpid.output2*3.1415926/180;
        upd();//修改
        servo(angle);//angle>pi时-pi
        delay(1000);
    }

}