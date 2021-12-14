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

  cv_bridge
  cmake_modules
  cv_bridge
  eigen_conversions
  image_geometry
  image_transport
  message_filters
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(identify_case src/identify_case.cpp)
target_link_libraries(identify_case ${catkin_LIBRARIES})

add_executable(identify_calendar src/identify_calendar.cpp)
target_link_libraries(identify_calendar ${catkin_LIBRARIES})

add_executable(transparent_detection src/transparent_detection.cpp)
target_link_libraries(transparent_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
