cmake_minimum_required(VERSION 2.8.3)
project(videoray_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  message_generation 
  sensor_msgs 
  syllo_common
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Throttle.msg
  DesiredTrajectory.msg
  AccessoryControl.msg
  Status.msg
  UHRIComm.msg
  RTI01.msg
  RTI02.msg
  RTI03.msg
  RTI30.msg
  RTI31.msg
  RTI32.msg
  RTI33.msg
  NavData.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES videoray
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(syllo_serial
  src/syllo_serial/serialib.cpp
)

add_executable(videoray_control 
  src/videoray_control/main.cpp 
  src/videoray_comm/Packetizer.cpp 
  src/videoray_comm/VideoRayComm.cpp  
)

add_dependencies(videoray_control
  videoray_control_generate_messages_cpp
  videoray_control_gencpp
)

target_link_libraries(videoray_control
  ${catkin_LIBRARIES}
  syllo_serial
)
