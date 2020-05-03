cmake_minimum_required(VERSION 2.8.3)
project(mrs_uav_general)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  nodelet
  std_msgs
  mavros_msgs
  )

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES AutomaticStart
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
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
