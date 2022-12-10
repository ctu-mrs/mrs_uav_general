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
  topic_tools
  )

add_definitions(-D_FILE_OFFSET_BITS=64)

set(LIBRARIES
  AutomaticStart mrs_rosbag
  )

set(EXECUTABLES
  mrs_record
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs mrs_msgs mrs_lib rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp
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

add_executable(mrs_record
  src/record.cpp
  )

target_link_libraries(mrs_record
  mrs_rosbag
  )

add_dependencies(AutomaticStart
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AutomaticStart
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY scripts/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
