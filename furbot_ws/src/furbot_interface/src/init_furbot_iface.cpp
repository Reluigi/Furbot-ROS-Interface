#include "init_furbot_iface.h"



int main (){
    // Load configuration
    int port_number = 3425;
    // Try to connect

    int sock;
    struct sockaddr_in addr;
    char buf[1024];
    int bytes_read;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        perror("socket create");
        exit(1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_number);
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // specify to listen on all IP addresses
    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) // explicitly bind socket with particular address
    {
        perror("socket bind");
        exit(2);
    }

    while(1)
    {
        bytes_read = recvfrom(sock, buf, 1024, 0, NULL, NULL);
        buf[bytes_read] = '\0';
        printf(buf);
    }

    // Check statuses of systems in status frame

    // Fork status publisher

    // Fork control subscriber

    // Wait for processes to finish, free memory

    return 0;
}