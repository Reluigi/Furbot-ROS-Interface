#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <unistd.h>

int main() {
    int sock;
    struct sockaddr_in addr;
    char buf[265];
    int bytes_read;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        perror("socket");
        exit(1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(0x4653);
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // specify to listen on all IP addresses
    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) // explicitly bind socket with particular address
    {
        perror("bind");
        exit(2);
    }

    while(1)
    {
        bytes_read = recvfrom(sock, buf, 256, 0, NULL, NULL);
        buf[bytes_read] = '\0';
        std::cout << "Bytes read = " << bytes_read << ", message: " << buf << std::endl;
    }
}

