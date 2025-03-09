#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <unistd.h>  
#include <arpa/inet.h>  
#include <signal.h>  
#include <termios.h>  
#include <fcntl.h>  

#define PORT 8888  
#define BUFFER_SIZE 1024

int kbhit(void) 
{  
    struct termios oldt, newt;  
    int oldf;  
    char ch;  
    tcgetattr(STDIN_FILENO, &oldt);  
    newt = oldt;  
    newt.c_lflag &= ~(ICANON | ECHO);  
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
    
    if (read(STDIN_FILENO, &ch, 1) >= 0) {  
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
        fcntl(STDIN_FILENO, F_SETFL, oldf);  
        return ch;  
    }  
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
    fcntl(STDIN_FILENO, F_SETFL, oldf);  
    return 0;  
}  

int main() 
{  
    //开启摄像头
    pid_t pid;
    pid = fork();  
    if (pid < 0) 
    {  
        perror("Fork failed");  
        return 1;  
    } 
    else if (pid == 0) 
    {  
        execlp("python3", "python3", "camera5.py", (char *)NULL);  
        perror("execlp failed");  
        exit(1);  
    }  
    
    int server_socket, client_socket;  
    struct sockaddr_in server_addr, client_addr;  
    socklen_t addr_len = sizeof(client_addr);  
    char buffer[BUFFER_SIZE] = {0};  
    char socket_angle[] = ";0,0,0;";
    float Acc[3], Gyro[3], Angle[3];
    int n=1;
    
    //连接socket服务器
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) 
    {  
        perror("socket");  
        exit(EXIT_FAILURE);  
    }  

    int opt = 1;  
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) 
    {  
        perror("setsockopt");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = INADDR_ANY;  
    server_addr.sin_port = htons(PORT);  

    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) 
    {  
        perror("bind");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    if (listen(server_socket, 1) < 0) {  
        perror("listen");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("waiting for connection...\n");  

    if ((client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &addr_len)) < 0) {  
        perror("accept");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("connected with: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));  
    const char *response = "Hello";  
    send(client_socket, response, strlen(response), 0);  
  
    while (1) 
    {
        printf("第%d次循环\n",n);
        if (kbhit()) 
        {  
            char key = getchar();  
            if (key == 'q' || key == 'Q') break;
        }  
        getparam(Acc, Gyro, Angle);
        sprintf(socket_angle, ";%f,%f,%f;", Angle[2], Angle[1], Angle[0]);
        printf("angle=%s",socket_angle);
        memset(buffer, 0, BUFFER_SIZE);  
        ssize_t bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0);  
        if (bytes_received <= 0) 
        {
            printf("failed to recieve, quiting...");
            break;
        }
        buffer[bytes_received] = '\0';
        // if (strcmp(buffer, "exit") == 0) 
        // {  
        //     printf("client asked for break\n");  
        //     break;  
        // }
        printf("received: %s\n", buffer);  
        if (send(client_socket, socket_angle, strlen(socket_angle), 0) <= 0)   
            printf("failed to send: %s\n", socket_angle);  
        else 
            printf("sent: %s\n", socket_angle);
            
        delay(100);
        n++;
    }  

    // 关闭连接  
    printf("closing camera\n");  
    if (pid > 0) kill(pid, SIGTERM); 
    close(client_socket);  
    close(server_socket);  
    return 0;  
}