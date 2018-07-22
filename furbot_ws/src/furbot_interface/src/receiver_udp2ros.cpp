/* This node listens on UDP port for FURBOT status frames.
 * It converts all telemetry received in to ROS topics.
 */

#include "furbot_interface/furbot_protocol.h"
#include "furbot_msgs/TractionData.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdint> // uint8_t
#include <cstdlib>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <pthread.h>
#include "ros/ros.h"

// Port number and IP address
int port = 0x4653;
unsigned long int address = INADDR_ANY;

// ROS params
int pub_freq = 100; //Hz

void * UdpThread(void *arg);

pthread_mutex_t status_mutex  = PTHREAD_MUTEX_INITIALIZER;

int main(int argc, char **argv){

    // ROS
    ros::init(argc, argv, "furbot_udp2ros");
    ros::NodeHandle nh;
    ros::Rate loop_rate(pub_freq);
    ros::Publisher chatter_pub = nh.advertise<furbot_msgs::TractionData>("~traction_data", 10);

    // Thread
    pthread_t udp_th;

    // Initialize status structures
    TractionStruct traction_status;
    SteeringStruct steering_status;

    StatusStruct status;
    status.traction_part = &traction_status;
    status.steering_part = &steering_status;

    if (pthread_create(&udp_th, nullptr, UdpThread, static_cast<void *>(&status))){
        perror("pthread_create error");
        std::exit(1);
    }

    if (pthread_detach(udp_th))
    {
        std::perror("pthread_detach error");
        std::exit(1);
    }

//    UdpThread();
    while(true){
        furbot_msgs::TractionData traction_msg;
        pthread_mutex_lock( &status_mutex );

        pthread_mutex_unlock( &status_mutex );
    }
    return 0;
}

void * UdpThread(void *arg){

    StatusStruct * status = (StatusStruct * ) arg;

    int sock;
    struct sockaddr_in addr;
    int bytes_read;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        perror("Socket create error");
        std::exit(1);
    }

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(address); // specify to listen on all IP addresses
    if(bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) // explicitly bind socket with particular address
    {
        perror("Socket bind error");
        std::exit(1);
    }

    bool fail = false;
    while (not fail){
        char buf[STATUS_FRAME_BUFFER_SIZE];
        bytes_read = recvfrom(sock, buf, STATUS_FRAME_BUFFER_SIZE, 0, NULL, NULL);
//        buf[bytes_read] = '\0';
//        std::cout << "Bytes read = " << bytes_read << ", message: " << buf << std::endl;
        std::cout << "12th byte: " << (int)buf[12] << "\n";
        pthread_mutex_lock( &status_mutex );
        if (ParseStatusFrame(buf, bytes_read, status)){
            std::cout << "Parsing error\n";
        }
        pthread_mutex_unlock( &status_mutex );
    }

    close(sock);
}
