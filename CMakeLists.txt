cmake_minimum_required(VERSION 3.0.2)
project(liquid_height_estimation)
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  message_generation
)

find_package(PCL 1.8 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/HeightDetector.cfg
)

add_message_files(
  FILES
  Statistics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES liquid_height_estimation
 CATKIN_DEPENDS roscpp rospy std_msgs tf message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(liquid_height_detection
  src/HeightDetector.cpp)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(height_detector src/height_detector_node.cpp src/HeightDetector)
target_link_libraries(height_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(height_detector ${PROJECT_NAME}_gencfg)