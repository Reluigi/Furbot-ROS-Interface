/* This node listens on UDP port for FURBOT status frames.
 * It converts all telemetry received in to ROS topics.
 */

#include "furbot_interface/furbot_protocol.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdint> // uint8_t
#include <cstdlib>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>

int main(int argc, char **argv){

    // Port number and IP address
    int port = 0x4653;
    unsigned long int address = INADDR_ANY;

    int sock;
    struct sockaddr_in addr;
    int bytes_read;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        perror("socket");
        std::exit(1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(address); // specify to listen on all IP addresses
    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) // explicitly bind socket with particular address
    {
        perror("bind");
        std::exit(1);
    }

    // Initialize status structures
    TractionStruct traction_status;
    SteeringStruct steering_status;

    StatusStruct status;
    status.traction_part = &traction_status;
    status.steering_part = &steering_status;

    std::cout << "Start while loop\n";
    bool fail = false;
    while (not fail){
        char buf[STATUS_FRAME_BUFFER_SIZE];
        bytes_read = recvfrom(sock, buf, STATUS_FRAME_BUFFER_SIZE, 0, NULL, NULL);
//        buf[bytes_read] = '\0';
//        std::cout << "Bytes read = " << bytes_read << ", message: " << buf << std::endl;
        std::cout << "12th byte: " << (int)buf[12] << "\n";
        if (ParseStatusFrame(buf, bytes_read, &status)){
            continue;
        }
        else {
            std::cout << "Speed: " << status.traction_part->speed << std::endl;
        }
    }

    close(sock);

    return 0;
}

