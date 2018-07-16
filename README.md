# Furbot ROS Interface

## Description of architecture
Overall organisation and reasons

## UDP-ROS nodes
Part of package which converts UDP status frame to ROS msgs with telemetry 

### Init
Initialization procedure with check of Furbot state.

### Status publisher
Take UDP status frame, convert them and publish to ROS topics.

### Control subscriber
Subscribe to ROS topic with control signals, convert them and send via UDP.

## Twist to Control node
Subscribe to topic with `std_msgs/Twist` type of control messages, run internal PID loops and publish control signals as ROS message. 

## Odometry calculation node
Using telemetry calculate odometry transformation.