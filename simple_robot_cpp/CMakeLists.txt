cmake_minimum_required(VERSION 3.0.2)
project(simple_robot_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  simple_robot_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_robot_cpp
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(motion_controller src/motion_controller.cpp)
target_link_libraries(motion_controller ${catkin_LIBRARIES})
add_dependencies(motion_controller ${catkin_EXPORTED_TARGETS})

add_executable(obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES})
add_dependencies(obstacle_detector ${catkin_EXPORTED_TARGETS})