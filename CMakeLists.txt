cmake_minimum_required(VERSION 3.1.2)
project(mrs_uav_general)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp)
find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options regex thread)
find_package(BZip2 REQUIRED)
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

add_definitions(-D_FILE_OFFSET_BITS=64)

catkin_package(
  #  INCLUDE_DIRS include
  LIBRARIES AutomaticStart mrs_rosbag
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs mrs_msgs mavros_msgs mrs_lib rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${BZIP2_INCLUDE_DIR}
)

add_library(AutomaticStart
  src/automatic_start.cpp)

add_library(mrs_rosbag
  src/player.cpp
  src/recorder.cpp
  src/time_translator.cpp)

target_link_libraries(mrs_rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${BZIP2_LIBRARIES}
)

add_executable(mrs_record src/record.cpp)
target_link_libraries(mrs_record mrs_rosbag)

add_dependencies(AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStart
  ${catkin_LIBRARIES}
  )
