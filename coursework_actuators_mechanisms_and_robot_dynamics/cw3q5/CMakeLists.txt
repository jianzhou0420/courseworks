cmake_minimum_required(VERSION 2.8.3)
project(cw3q5)
find_package(catkin REQUIRED COMPONENTS

  rosbag
  trajectory_msgs
  rospy
  sensor_msgs
  cw3q2
)
catkin_package()

#include_directories(
#	${catkin_INCLUDE_DIRS}
#)

#add_executable(cw3q5 src/cw3q5.py)
#target_link_libraries(cw3q5 ${catkin_LIBRARIES})

