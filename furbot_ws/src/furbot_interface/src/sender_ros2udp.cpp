/* This node subscribes to control signals' topic port,
 * packs data in UDP frame and sends it to Furbot.
 */

#include "furbot_interface/furbot_protocol.h"
#include "furbot_msgs/ControlSignals.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <string>
#include "ros/ros.h"

class SubWithSocket{
    int sock;
    struct sockaddr_in addr;
    ros::Subscriber sub;
public:
    SubWithSocket(ros::NodeHandle* nh, int port, unsigned long int address, std::string topic);
    ~SubWithSocket();
    void SubCallback(const furbot_msgs::ControlSignals::ConstPtr& msg);
};

SubWithSocket::SubWithSocket(ros::NodeHandle* nh, int port, unsigned long int address, std::string topic){
    // create socket
    this->sock = socket(AF_INET, SOCK_DGRAM, 0); // create socket for datagrams (UDP) in Internet address space
    if(this->sock < 0)
    {
        perror("Socket create error");
        exit(1);
    }

    addr.sin_family = AF_INET; // address space - Internet
    addr.sin_port = htons(port); // convert Port number from host byte order to network byte order
    addr.sin_addr.s_addr = htonl(address);

    // subscribe
    this->sub = nh->subscribe(topic, 1, &SubWithSocket::SubCallback, this);
}

SubWithSocket::~SubWithSocket() {
    close(this->sock);
}

void SubWithSocket::SubCallback(const furbot_msgs::ControlSignals::ConstPtr &msg) {
    std::string remote_frame;

    // Combine msg starting with magic word
    // TODO: make function in furbot_protocol.h to fill first 2 fields
//    remote_frame.append(magic, sizeof(char)*4);

    // Timestamp
    timeval now;
    gettimeofday(&now, NULL);
    uint32_t timestamp = (now.tv_sec - 1531000000) * 1000 + now.tv_usec/1000;
    uint32_t timestamp_net = htonl(timestamp);
    remote_frame.append((const char *) &timestamp_net, sizeof(uint32_t));
}

int main (int argc, char** argv){
    ros::init(argc, argv, "furbot_ros2udp");
    ros::NodeHandle nh;
    std::string topic = "control_signals";

    int port = 0x4654;
    unsigned long int address = INADDR_LOOPBACK;

    SubWithSocket sender(&nh, port, address, topic);

    ros::spin();

    return 0;
}
