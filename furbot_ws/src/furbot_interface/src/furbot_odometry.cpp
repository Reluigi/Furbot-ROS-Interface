#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  
  //getting traction data subscribing to TractionData message
  ros::Subscriber sub = n.subscribe("TractionData", 1000, chatterCallback);

  //furbot specification
  double radius = 0.3; //wheel radius
  double L = 0.6425; //distance between wheel and origin of robot frame
  double pi = 3.1415926535;

  
  //v = 2πr × RPM × (60/1000) km/hr

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  
//assuming furbot works in rad for angles

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    /*
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    */

    //converting rear speeds in linear velocities (from RPM to m/s)
    // m/s = 60*RPM/(4*pi^2*r)
    double vel_r_lin = (60*sub.vel_r)/(4*pi*pi*radius);
    double vel_l_lin = (60*sub.vel_l)/(4*pi*pi*radius);

    //compute v and w using matlab formulas (challenge emaro days)
    double v = (vel_r_lin + vel_l_lin)/2;
    double w = (vel_l_lin - vel_r_lin)/L;

    /*
    //furbot [v, w] computation
    double v = th * r; //phi1*r/2 + phi2*r/2
    double w = th * r / L;
    */

    //furbot odometry equations
    double dt = (current_time - last_time).toSec();
    double x_new = x + v*dt*cos(th);
    double y_new = y + v*dt*sin(th);
    double th_new = th + w*dt;

    x = x_new;
    y = y_new;
    th = th_new;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
