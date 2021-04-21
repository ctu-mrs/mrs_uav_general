cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_general)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  mavros_msgs
  topic_tools
  )

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES AutomaticStart
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib
  )

include_directories(
  ${catkin_INCLUDE_DIRS})

# automatic start

add_library(AutomaticStart
  src/automatic_start.cpp)

add_dependencies(AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStart
  ${catkin_LIBRARIES}
  )
