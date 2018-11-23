cmake_minimum_required(VERSION 2.8.3)
project(furbot_testing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_executable(status_sender status_sender.cpp)

add_executable(remote_receiver remote_receiver.cpp)
