cmake_minimum_required(VERSION 2.8.3)
project(videoray_input)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  syllo_common
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs syllo_common
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(videoray_input
  src/videoray_input/main.cpp
  src/joystick/Joystick.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(videoray_input
  ${catkin_LIBRARIES}
)
