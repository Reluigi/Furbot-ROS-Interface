/* This node subscribes to control signals' topic,
 * packs data in UDP frame and sends it to Furbot.
 */

#include "furbot_interface/furbot_protocol.h"
#include "furbot_msgs/ControlSignals.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstdint>
#include <cstdlib>
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
    StampRemoteFrame(&remote_frame);

    int32_t steer =  msg->steer;
    int32_t throttle =  msg->throttle;
    uint32_t brake =  msg->brake;

    remote_frame.append((const char *) &steer, sizeof(int32_t));
    remote_frame.append((const char *) &throttle, sizeof(int32_t));
    remote_frame.append((const char *) &brake, sizeof(uint32_t));

    sendto(sock, &remote_frame[0], remote_frame.size(), 0, (struct sockaddr *)&addr, sizeof(addr));
}

int main (int argc, char** argv){
    ros::init(argc, argv, "furbot_ros2udp");
    ros::NodeHandle nh;
    std::string topic = "furbot/control_signals";

    int port = 0x4654;
    unsigned long int address = INADDR_LOOPBACK;

    SubWithSocket sender(&nh, port, address, topic);

    ros::spin();

    return 0;
}
