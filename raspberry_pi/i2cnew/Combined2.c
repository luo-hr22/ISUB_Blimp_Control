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

//#define会报错，请定义变量
int asbst = 0;//0代表树莓派向后；1代表树莓派向前
int pin_s=26;//舵机
int pin_r1 = 1;//右侧电机正转信号
int pin_r2 = 2;//右侧电机反转信号
int pin_l1 = 22;//左侧电机正转信号
int pin_l2 = 23;//左侧电机反转信号
//引脚位置注意查证

char key;
int speed_r = 0;//虚拟速度, 范围:[-1000,1000]
int speed_l = 0;//^^^^^^^
int aver = 0;//左右平均速度
int mark = 0;//1代表临时停止
int r_last, l_last;//用于暂停程序及其恢复
float angleRAD;//弧度制角度
int angle;//角度制角度
int va;//虚拟角度

int status = -1;//1代表手动控制，0代表自动控制，-1代表待机
int marker = 0;//1代表可能输入了手动操作指令


float Acc[3], Gyro[3], Angle[3];
float feedbackValue;
float targetValue;
float axvalue;
float azvalue;
PID mypid = { 0 };
PID yourpid = { 0 };

//PID调参区
float maxiV = 100;
float maxiAngle = 360;
float maxiI = 1000000000;
float maxiI2 = 1000000000;
float k0[] = { 00, 0, 0 };//kp,ki,kd for mypid
float k1[] = { 20, 0, 0 };//kp,ki,kd for yourpid
float k2[] = { 20, 0, 0 };//kp2,ki2,kd2 for yourpid



void printscreen(void)
{
    printf("Speed_L=%d, Speed_R=%d, Angle=%ddegree\n",speed_l, speed_r, angle * 180 / 3.1415926);
}

void calva(void)
{
    int temp = angle;
    if (temp >= 180)
        temp -= 180;
    if (asbst == 0)
        va = ((180.0 - temp) / 9.0 + 5) * 10;
    else
        va = (temp / 9.0 + 5) * 10;
}

void servo(void)
{
    calva(angle);
    pwmWrite(pin_s, va);
    printscreen();
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

int kbhit(void)
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
}

void upd(void)
{
    motors(pin_r1, pin_r2, speed_r);
    motors(pin_l1, pin_l2, speed_l);
    aver = 0.5 * (speed_r + speed_l);
    printscreen();
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
    aver = 0;
    angle = asbst * 180;
    angleRAD = asbst * 3.1415926;
}

void PID_control(void)
{
    getparam(Acc, Gyro, Angle);

    feedbackValue = Angle[2];
    targetValue = 0;
    //axvalue = Acc[0] /16384.0 ;
    //azvalue = Acc[2] / 16384.0;

    PID_Calc(&mypid, targetValue, feedbackValue);
    PID_Calc2(&yourpid, angle, abs(aver), Acc[0], Acc[2] - 1);
    //yourpid.output2为角度制，其在大于180度后电机反转,舵机转动度数减180
    if (yourpid.output2 >= 180) aver = -yourpid.output;
    else aver = yourpid.output;
    speed_l = aver + mypid.output;
    speed_r = aver - mypid.output;
    if (speed_l>100) speed_l=100;
    if (speed_r>100) speed_r=100;
    angleRAD = yourpid.output2;
    angle = angleRAD * 3.1415926 / 180;
    upd();
    servo();
    delay(500);
}

void Manual_control(void)
{
    if (marker == 1)
    {
        marker = 0;
        switch (key)
        {
        case 'n':
        case 'N':
            if (mark == 1)
            {
                speed_r = r_last;
                speed_l = l_last;
                mark = 0;
                upd();
            }
            break;
        case 'w':
        case 'W':
            if (speed_l < 100 && speed_r < 100)
            {
                speed_l++; speed_r++;
                upd();
            }
            break;
        case 's':
        case 'S':
            if (speed_l > -100 && speed_r > -100)
            {
                speed_l--; speed_r--;
                upd();
            }
            break;
        case 'a':
        case 'A':
            if (speed_l > -100)
                speed_l--;
            if (speed_r < 100)
                speed_r++;
            upd();
            break;
        case 'd':
        case 'D':
            if (speed_r > -100)
                speed_r--;
            if (speed_l < 100)
                speed_l++;
            upd();
            break;
        case 'b':
        case 'B':
            r_last = speed_r;
            l_last = speed_l;
            speed_r = 0; speed_l = 0;
            mark = 1;
            upd();
            break;
        case 'j':
        case 'J':
            if (angle < 176)
                angle += 5;
            servo(angle);
            break;
        case 'u':
        case 'U':
            if (angle > 4)
                angle -= 5;
            servo(angle);
            break;
        case 'p':
        case 'P':
            ini();
            break;
        case ' ':
            aver = (speed_r + speed_l) / 2;
            speed_r = aver; speed_l = aver;
            upd();
            break;
        }
    }
}

int main()
{
    //初始化
    if (ini() == -1)
        return -1;
    PID_Init(&mypid, k0[0], k0[1], k0[2], maxiI, maxiV);
    PID_Init2(&yourpid, k1[0], k1[1], k1[2], maxiI, maxiV, k2[0], k2[1], k2[2], maxiI2, maxiAngle);
    printf("Enter to start\n");
    fflush(stdin);
    scanf("");
    
    do
    {   
        if (kbhit())
        {
            key = getchar();
            switch (key)
            {
            case 'C':
            case 'c':
                status = 0;
                marker = 0;
                break;
            case 'M':
            case 'm':
                status = 1;
                marker = 0;
                break;
            case 'O':
            case 'o':
                status = -1;
                marker = 0;
                ini();
                printf("Manual or Auto control stopped\nPlease choose a mod\nM:Manual; C:Auto\nor press Q to exit");
            case 'q':
            case 'Q':
                goto aaa;
            default:
                marker = 1;
            }
        }
        else 
            marker = 0;

        if (status == 1)//手动控制
            Manual_control();
        else if (status == 0)//自动控制
            PID_control();

    } while (1);
 aaa:
    printf("Exited\n");
    return 0;

}
