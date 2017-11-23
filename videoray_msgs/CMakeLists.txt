cmake_minimum_required(VERSION 2.8.3)
project(videoray_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  NavData.msg
  ScanLine.msg
  Throttle.msg
  USBLPosition.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  std_msgs
  message_runtime
)