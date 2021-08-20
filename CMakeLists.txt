cmake_minimum_required(VERSION 2.8.3)
project(apu_medication_support)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  sensor_msgs
  std_msgs
  pcl_conversions
  tf
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(identify_case src/identify_case.cpp)
target_link_libraries(identify_case ${catkin_LIBRARIES})
