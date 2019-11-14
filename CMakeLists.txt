cmake_minimum_required(VERSION 2.8.3)
project(medicine_calendar)

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

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(medicine_calendar_pcl src/medicine_calendar.cpp)
target_link_libraries(medicine_calendar_pcl ${catkin_LIBRARIES})
