/* This node subscribes to cmd_vel topic (where any higher level controller
 * publishes Twist.msg). It runs low level PID control loops, and publishes
 * ControlSignals.msg to furbot/control_signals topic.
 */

#include "ros/ros.h"
#include "furbot_interface/furbot_protocol.h"
#include "furbot_interface/PidControl.h"
#include "geometry_msgs/Twist.h"
#include "furbot_msgs/TractionData.h"
#include "furbot_msgs/ControlSignals.h"

/*
 * Class to keep PIDs, subscriber, publishers, and their common
 * data together.
 */
class ControlAdapter{
    ros::Publisher control_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber traction_sub;
    TractionStruct traction_data;

    PidControl throttle_pid;
    PidControl brake_pid;

public:
    ControlAdapter(ros::NodeHandle* nh, std::string traction_topic, std::string cmd_topic, std::string control_topic);
    ~ControlAdapter();

    /**
     * Store Traction Data in traction_data
     * @param msg Default ROS arg of corresponding msg type.
     */
    void TractionSubCallback(const furbot_msgs::TractionData::ConstPtr& msg);

    /**
     * Grab Twist control msg, use PIDs to calculate and publish control signals.
     * @param msg Default ROS arg of corresponding msg type.
     */
    void CmdVelSubCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

int main(int argc, char** argv){
    ros::init(argc, argv, "furbot_twist2control");
    ros::NodeHandle nh;
    std::string traction_topic = "furbot/traction_data";
    std::string control_topic = "furbot/control_signals";
    std::string cmd_topic = "cmd_vel";

    return 0;
}

ControlAdapter::ControlAdapter(ros::NodeHandle *nh, std::string traction_topic, std::string cmd_topic,
                               std::string control_topic) {
    // TODO: init subs, pub, pids
    traction_sub = nh->subscribe(traction_topic, 1, &ControlAdapter::TractionSubCallback, this);
    cmd_vel_sub = nh->subscribe(cmd_topic, 1, &ControlAdapter::TractionSubCallback, this);
    control_pub = nh->nh.advertise<furbot_msgs::ControlSignals>(control_topic, 1);

}

void ControlAdapter::CmdVelSubCallback(const geometry_msgs::Twist::ConstPtr &msg) {

    // Grab pid parameters in loop after publishing

}

void ControlAdapter::TractionSubCallback(const furbot_msgs::TractionData::ConstPtr &msg) {
    traction_data.state = msg->state;
    traction_data.mode = msg->mode;
    traction_data.speed = msg->speed;
    traction_data.vel_l = msg->vel_l;
    traction_data.vel_r = msg->vel_r;
    traction_data.throttle = msg->throttle;
    traction_data.brake = msg->brake;
    traction_data.reverse_flag = msg->reverse_flag;
    traction_data.odo_travel = msg->odo_travel;
}
