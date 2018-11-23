# Furbot ROS Interface

Furbot is a vehicle made in University of Genova. It is designed to be manually controlled by an operator, but it has an interface to control it via special protocol. It will be more useful to make Furbot autonomous at some level.
Our project is part of a platform project, this means that more than one group works to the same robot in order to make a coordinate work. Our main goal was to develop a ROS interface for Furbot vehicle, in order to make it able to be controllable via ROS.

**A detailed report of the project is available in** `report.pdf`.

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