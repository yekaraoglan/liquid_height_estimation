cmake_minimum_required(VERSION 3.0.2)
project(liquid_height_estimation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES liquid_height_estimation
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

