cmake_minimum_required(VERSION 2.8.3)
project(tritech_micron)

set(CMAKE_CXX_FLAGS "-std=c++0x -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  pcl_ros
  videoray_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp videoray_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/tritech_micron/tritech_micron.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(scanline_to_pointcloud
  src/tritech_micron/scanline_to_pointcloud.cpp
)

target_link_libraries(scanline_to_pointcloud
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} scanline_to_pointcloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)