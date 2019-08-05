cmake_minimum_required(VERSION 2.8.3)
project(mrs_general)

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
  LIBRARIES AutomaticStartDarpa AutomaticStartEagle AutomaticStartPointer
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

# automatic start MBZIRC

add_executable(automatic_start_mbzirc
  src/automatic_start_mbzirc.cpp)

add_dependencies(automatic_start_mbzirc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(automatic_start_mbzirc
  ${catkin_LIBRARIES}
  )

# automatic start DARPA

add_library(AutomaticStartDarpa
  src/automatic_start_darpa.cpp)

add_dependencies(AutomaticStartDarpa
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStartDarpa
  ${catkin_LIBRARIES}
  )

# automatic start EAGLE

add_library(AutomaticStartEagle
  src/automatic_start_eagle.cpp)

add_dependencies(AutomaticStartEagle
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStartEagle
  ${catkin_LIBRARIES}
  )

# automatic start POINTER

add_library(AutomaticStartPointer
  src/automatic_start_pointer.cpp)

add_dependencies(AutomaticStartPointer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStartPointer
  ${catkin_LIBRARIES}
  )
