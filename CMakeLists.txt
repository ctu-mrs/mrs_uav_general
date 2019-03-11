cmake_minimum_required(VERSION 2.8.3)
project(mrs_general)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -fno-diagnostics-color")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mrs_msgs
  mrs_lib
  )

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES mrs_general
  CATKIN_DEPENDS
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

## Declare a C++ executable
add_executable(automatic_start
  src/automatic_start.cpp)

add_dependencies(automatic_start
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(automatic_start
  ${catkin_LIBRARIES}
  )
