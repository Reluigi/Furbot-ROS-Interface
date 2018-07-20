# Furbot ROS Interface
As part of platform Furbot project this project aims to provide ROS interface for Furbot.
The main ideas are to make interface fast, realiable, and compatible to current ROS standarts and conventions.

## Description of architecture
Overall organisation and reasons.

## UDP-ROS nodes
Part of package which converts UDP status frame to ROS msgs with telemetry.

### Init
Initialization procedure with check of Furbot state.

### Status publisher
Take UDP status frame, convert them and publish to ROS topics.

### Control subscriber
Subscribe to ROS topic with control signals, convert them and send via UDP.

## Twist to Control node
Param server for PID coefficients.

Subscribe to topic with `std_msgs/Twist` type of control messages, run internal PID loops and publish control signals as ROS message. 

## Odometry calculation node
Using telemetry calculate odometry transformation.

## Usage
Launch file description.

## Further work
Complete parsing functionality and add ROS msgs.