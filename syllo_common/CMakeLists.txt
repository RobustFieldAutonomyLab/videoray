cmake_minimum_required(VERSION 2.8.3)
project(syllo_common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES syllo_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(syllo_common
  src/syllo_common/SylloNode.cpp
  src/syllo_common/Orientation.cpp
  src/syllo_common/Filter.cpp
  src/syllo_common/Utils.cpp
)
target_link_libraries(syllo_common
  ${catkin_LIBRARIES}
)
