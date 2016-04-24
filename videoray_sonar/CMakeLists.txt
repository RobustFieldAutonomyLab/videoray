cmake_minimum_required(VERSION 2.8.3)
project(videoray_sonar)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(tritech_micron
  src/tritech_micron/tritech_micron.cpp
)
add_library(serial
  src/serial/Serial.cpp
)

add_executable(videoray_sonar
  src/videoray_sonar/videoray_sonar.cpp
)

target_link_libraries(videoray_sonar
  tritech_micron
  serial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)