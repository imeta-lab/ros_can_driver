cmake_minimum_required(VERSION 3.0.2)
project(ros_msgs)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "--std=c++17 -O0 -Og -ggdb")
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  # geometry_msgs
)

add_message_files(
  DIRECTORY can_msg
  FILES
  ControlCommand.msg
  MotorCommand.msg
  MotorState.msg
  MotorStateArray.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  # geometry_msgs
)

catkin_package()