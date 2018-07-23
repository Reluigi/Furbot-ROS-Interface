/* This node subscribes to cmd_vel topic (where any higher level controller
 * publishes Twist.msg). It runs low level PID control loops, and publishes
 * ControlSignals.msg to furbot/control_signals topic.
 */

#include "ros/ros.h"
#include "furbot_interface/furbot_protocol.h"
#include "geometry_msgs/Twist.h"
#include "furbot_msgs/TractionData.h"
#include "furbot_msgs/ControlSignals.h"

class ControlAdapter{
    ros::Publisher control_pub;
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber traction_sub;
    TractionStruct traction_data;
    // TODO: PIDs
public:
    ControlAdapter(ros::NodeHandle* nh, std::string traction_topic, std::string cmd_topic, std::string control_topic);
    ~ControlAdapter();
    void TractionSubCallback(const furbot_msgs::TractionData::ConstPtr& msg);
    void CmdVelSubCallback(const geometry_msgs::Twist::ConstPtr& msg);
};

int main(int argc, char** argv){
    ros::init(argc, argv, "furbot_twist2control");
    ros::NodeHandle nh;
    std::string traction_topic = "furbot/traction_data";
    std::string control_topic = "furbot/control_signals";

    return 0;
}

