cmake_minimum_required(VERSION 2.8.3)
project(furbot_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  furbot_msgs
  nav_msgs
)

find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES furbot_interface
  CATKIN_DEPENDS roscpp std_msgs furbot_msgs geometry_msgs nav_msgs
#  DEPENDS pthread
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(receiver_udp2ros_node src/receiver_udp2ros.cpp src/furbot_protocol.cpp)
target_link_libraries(receiver_udp2ros_node ${catkin_LIBRARIES})
add_dependencies(receiver_udp2ros_node ${catkin_EXPORTED_TARGETS})

if(THREADS_HAVE_PTHREAD_ARG)
    target_compile_options(receiver_udp2ros_node PUBLIC "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
    target_link_libraries(receiver_udp2ros_node "${CMAKE_THREAD_LIBS_INIT}")
endif()


add_executable(furbot_odometry_node src/furbot_odometry.cpp)
target_link_libraries(furbot_odometry_node ${catkin_LIBRARIES})
