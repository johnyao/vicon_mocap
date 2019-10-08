cmake_minimum_required(VERSION 3.0.0)
project(vicon_odom)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_msgs
  vicon
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_executable(vicon_odom
  src/vicon_odom.cpp
  src/filter.cpp
)

add_executable(vicon_odom_simple
  src/vicon_odom_simple.cpp
)

add_dependencies(vicon_odom vicon_generate_messages_cpp)
add_dependencies(vicon_odom_simple vicon_generate_messages_cpp)

target_link_libraries(vicon_odom
  ${catkin_LIBRARIES}
)

target_link_libraries(vicon_odom_simple
  ${catkin_LIBRARIES}
)

install(TARGETS vicon_odom
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# There is a nicer way to do this if we use CMake 3.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
