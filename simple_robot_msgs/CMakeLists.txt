cmake_minimum_required(VERSION 3.0.2)
project(simple_robot_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  Obstacle.msg
)

add_service_files(
  FILES
  DriveVelocity.srv
  ToggleMode.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
