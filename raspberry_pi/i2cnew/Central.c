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
#include <stdlib.h>
#include <string.h>  
#include <arpa/inet.h> 
#include <signal.h> 

//#define会报错，请定义变量
#define BUFFER_SIZE 1024
#define PORT 8888  
#define pin_s 26 //舵机
#define pin_r1 29 //右侧电机正转信号
#define pin_r2 28 //右侧电机反转信号
#define pin_l1 21 //左侧电机正转信号
#define pin_l2 27 //左侧电机反转信号
#define asbst 0//0代表树莓派向后；1代表树莓派向前
//引脚位置注意查证
#define Freq 100
#define P 0.5
 
char key;
int speed_r = 0;//虚拟速度, 范围:[-1000,1000]
int speed_l = 0;//^^^^^^^
int aver = 0;//左右平均速度
short mark = 0;//1代表临时停止
int r_last, l_last;//用于暂停程序及其恢复
float angleRAD;//弧度制角度
int angle;//角度制角度
int va;//虚拟角度

short status = -1;//1代表手动控制，0代表自动控制，-1代表待机
short marker = 0;//1代表可能输入了手动操作指令
short oo = 0;//1代表打开相机定位函数开关
short C_on = 0;//0代表可能为上升沿

float Acc[3], Gyro[3];
float Angle[]={0,0,0};
float feedbackValue;
float targetValue;
float axvalue;
float azvalue;
PID mypid = { 0 };
PID yourpid = { 0 };
char socket_angle[1000];
double location[]={0, 0, 0};
double recv_info[]={0, 0, 0, 0};
double target_location[]={0, 0, 0};
int server_socket;
int client_socket;
struct sockaddr_in server_addr, client_addr;
socklen_t addr_len = sizeof(client_addr);
char buffer[BUFFER_SIZE] = { 0 };

//PID调参区
float maxiV = 100;
float maxiAngle = 360;
float maxiI = 1000000000;
float maxiI2 = 1000000000;
float k0[] = { 00, 0, 0 };//kp,ki,kd for mypid
float k1[] = { 20, 0, 0 };//kp,ki,kd for yourpid
float k2[] = { 20, 0, 0 };//kp2,ki2,kd2 for yourpid

pid_t pid;

int camera(void)
{
    pid = fork(); //
    if (pid < 0) 
    {  
        perror("Fork failed");  
        return 1;  
    } else if (pid == 0) {  
        //
        execlp("python3", "python3", "camera5.py", (char *)NULL);  
        perror("execlp failed");  
        exit(1);  
    }  

    // 
    //printf("Press 'q' to quit...\n");  
    /*while (1) 
    {  
        if(kbhit())
        {
            char key = getchar();        
            if (key == 'q') 
            {  
            printf("Quitting...\n");  
            cleanup();  
            break;  
            }  
        }
    }*/  
    return 0;  
}

void Pack(float* SET)
{
    sprintf(socket_angle, ";%f,%f,%f;", SET[2], SET[1], SET[0]);
}

void Unpack(double* SET, char* str)
{
    int i = 0;
    char* token = NULL;
    char temp2[1000];
    char temp[4][1000];
    strcpy(temp2,str);
    token = strtok(temp2, ";");
    token = strtok(token, ",");
    while (token != NULL)
    {
        strcpy(temp[i], token);
        token = strtok(NULL, ",");
        i++;
    }
    while (i > 0)
    {
        i--;
        SET[i] = strtod(temp[i], NULL);
    }
}

void sav(int type)
{
    double* x;
    switch (type)
    {
        case 1:
            x = location;
            break;
        case 2:
            x = target_location;
            break;
        default:
            return;
            break;
    }
    for (int i = 0; i < 3; i++)
        x[i] = recv_info[i + 1];
    printf("location: (%f,%f,%f)\n", location[0],location[1],location[2]);
    printf("target_location: (%f,%f,%f)\n", target_location[0],target_location[1],target_location[2]);
}
void printscreen(void)
{
    printf("Speed_L=%d, Speed_R=%d, Angle=%ddegree\n", speed_l, speed_r, angle);
}

void calva(void)
{
    int temp = angle;
    if (temp > 180)
        temp -= 180;
    if (asbst == 0)
        va = ((180.0 - temp) / 9.0 + 5) * 10;
    else
        va = (temp / 9.0 + 5) * 10;
}

void servo(void)
{
    angleRAD = angle * 3.1415926 / 180;
    calva();
    pwmWrite(pin_s, va);
    printscreen();
}

void motors(int pin1, int pin2, int v)
{
    if (v > 0)
    {
        softPwmWrite(pin1, v * P);
        softPwmWrite(pin2, 0);
    }
    else
    {
        softPwmWrite(pin1, 0);
        softPwmWrite(pin2, -v * P);
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

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

char Uncap(char st)
{
    if (st >= 'A' && st <= 'Z')
        st = st + 32;
    return st;
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
    upd();
    servo();
}

void PID_control(void)
{
    getparam(Acc, Gyro, Angle);
    feedbackValue = Angle[2];
    //axvalue = Acc[0] /16384.0 ;
    //azvalue = Acc[2] / 16384.0;

    PID_Calc(&mypid, targetValue, feedbackValue);
    PID_Calc2(&yourpid, angle, abs(aver), Acc[0], Acc[2] - 1);
    //yourpid.output2为角度制，其在大于180度后电机反转,舵机转动度数减180
    if (yourpid.output2 > 180) aver = -yourpid.output;
    else aver = yourpid.output;
    speed_l = aver + mypid.output;
    speed_r = aver - mypid.output;
    if (speed_l > 100) speed_l = 100;
    if (speed_r > 100) speed_r = 100;
    angleRAD = yourpid.output2;
    angle = angleRAD * 3.1415926 / 180;
    upd();
    servo();
    delay(Freq);
}

void Manual_control(void)
{
    if (marker == 1)
    {
        marker = 0;
        switch (key)
        {
        case 'n':
            if (mark == 1)
            {
                speed_r = r_last;
                speed_l = l_last;
                mark = 0;
                upd();
            }
            break;
        case 'w':
            if (speed_l < 100 && speed_r < 100)
            {
                speed_l += 5; speed_r += 5;
                upd();
            }
            break;
        case 's':
            if (speed_l > -100 && speed_r > -100)
            {
                speed_l -= 5; speed_r -= 5;
                upd();
            }
            break;
        case 'a':
            if (speed_l > -100)
                speed_l -= 5;
            if (speed_r < 100)
                speed_r += 5;
            upd();
            break;
        case 'd':
            if (speed_r > -100)
                speed_r -= 5;
            if (speed_l < 100)
                speed_l += 5;
            upd();
            break;
        case 'b':
            r_last = speed_r;
            l_last = speed_l;
            speed_r = 0; speed_l = 0;
            mark = 1;
            upd();
            break;
        case 'j':
            if (angle < 176)
                angle += 5;
            servo();
            break;
        case 'u':
            if (angle > 4)
                angle -= 5;
            servo();
            break;
        case 'p':
            ini();
            break;
        case 'z':
            break;
        case 'x':
            break;
        case ' ':
            aver = (speed_r + speed_l) / 2;
            speed_r = aver; speed_l = aver;
            upd();
            break;
        }
    }
}

void ini_socket(void)
{
    server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = INADDR_ANY;  
    server_addr.sin_port = htons(PORT);
     // 
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
    {  
        perror("socket");  
        exit(EXIT_FAILURE);  
    }  

    // 
    int opt = 1;  
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {  
        perror("setsockopt");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    // 
    server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = INADDR_ANY;  
    server_addr.sin_port = htons(PORT);  

    //  
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
    {  
        perror("bind");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    //   
    if (listen(server_socket, 1) < 0) 
    {  
        perror("listen");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("waiting for connection...\n");  

    // 
    if ((client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &addr_len)) < 0) 
    {  
        perror("accept");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("connected with: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));  
    //const char *response = "Hello";
    //send(client_socket, response, strlen(response), 0);
}

void Loc(void)
{
    Pack(Angle);
    printf("\n%f,%f,%f\n",Angle[2],Angle[1],Angle[0]);
    if (socket_angle != NULL)
        send(client_socket, socket_angle, strlen(socket_angle), 0);
    memset(buffer, 0, BUFFER_SIZE);  
    ssize_t bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0);  
    if (bytes_received <= 0) return; 
    buffer[bytes_received] = '\0';  
    if (strcmp(buffer, "exit") == 0) {  
        printf("client asked for break\n");  
        return;  
    }  
    //printf("received: %s\n", buffer);
    Unpack(recv_info, buffer);
    sav((int)recv_info[0]);
}

int main()
{
    //初始化
    //camera();
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
            marker = 0;
            key = Uncap(key);//大写转小写
            switch (key)
            {
            case 'c':
                targetValue = Angle[2];
                status = 0;
                break;
            case 'm':
                status = 1;
                break;
            case 'o':
                status = -1;
                ini();
                printf("Manual or PID control stopped\nPlease choose a mod\nM:Manual; C:PID\nor press Q to exit\n");
                break;
            case 'q':
                goto aaa;
                break;
            case 'l'://打开或关闭定位系统
                oo = 1 - oo;
                break;
            default:
                marker = 1;
                break;
            }
        }
        else
            marker = 0;

        if (status == 1)//手动控制
            Manual_control();
        else if (status == 0)//自动控制
            PID_control();
            
        if (oo == 1)
        {
            if (C_on == 0)
            {
                ini_socket();
                C_on = 1;
            }
            Loc();
        }
        else if (C_on == 1)
        {
            close(client_socket);
            close(server_socket);
            C_on = 0;
        }
            
    } while (1);
aaa:
    printf("Exited\n");
    return 0;

}
