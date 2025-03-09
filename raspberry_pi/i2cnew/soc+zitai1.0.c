#include <stdio.h>  
#include <stdlib.h>  
#include <string.h>  
#include <unistd.h>  
#include <arpa/inet.h>  

#define PORT_SOC 8888  
#define BUFFER_SIZE 1024  

int main() {  
    int server_socket, client_socket;  
    struct sockaddr_in server_addr, client_addr;  
    socklen_t addr_len = sizeof(client_addr);  
    char buffer[BUFFER_SIZE] = {0};  
    char socket_angle[BUFFER_SIZE] = ";0,0,0;"; // 默认值为0  
    float Acc[3], Gyro[3], Angle[3];  

    // 创建socket  
    if ((server_socket = socket(AF_INET, SOCK_STREAM, 0)) == -1) {  
        perror("socket");  
        exit(EXIT_FAILURE);  
    }  

    // 设置 socket 选项  
    int opt = 1;  
    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {  
        perror("setsockopt");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    // 设置服务器地址  
    server_addr.sin_family = AF_INET;  
    server_addr.sin_addr.s_addr = INADDR_ANY;  
    server_addr.sin_port = htons(PORT_SOC);  

    // 绑定socket到地址和端口  
    if (bind(server_socket, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) 
    {  
        perror("bind");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  

    // 开始监听  
    if (listen(server_socket, 1) < 0) 
    {  
        perror("listen");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("waiting for connection...\n");  

    // 接受连接  
    if ((client_socket = accept(server_socket, (struct sockaddr *)&client_addr, &addr_len)) < 0) 
    {  
        perror("accept");  
        close(server_socket);  
        exit(EXIT_FAILURE);  
    }  
    printf("connected with: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port)); // 修正为sin_port  
    const char *response = "Hello";  
    send(client_socket, response, strlen(response), 0);  
    
    // 接收数据  
    while (1) 
    {  
        getparam(Acc, Gyro, Angle); // 获取数据  
        sprintf(socket_angle, ";%f,%f,%f;", Angle[2], Angle[1], Angle[0]);
        printf("angle=%s\n", socket_angle);  
        memset(buffer, 0, BUFFER_SIZE);  

        ssize_t bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0);  
        if (bytes_received <= 0) break;   
        buffer[bytes_received] = '\0';  
        if (strcmp(buffer, "exit") == 0) 
        {  
            printf("client asked for break\n");  
            break;  
        }  
        printf("received: %s\n", buffer);  
 
        if (send(client_socket, socket_angle, strlen(socket_angle), 0) <= 0)   
            printf("failed to send: %s\n", socket_angle);  
         else 
            printf("sent: %s\n", socket_angle);  
    }  

    // 关闭连接  
    close(client_socket);  
    close(server_socket);  

    return 0;  
}
