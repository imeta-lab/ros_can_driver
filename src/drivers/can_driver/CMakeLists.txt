cmake_minimum_required(VERSION 3.10)
project(can_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "--std=c++17 -O0 -Og -ggdb")
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_msgs
)

find_package(Protobuf REQUIRED)
# generate *.pb.h and *.pb.cc of *.proto file
execute_process( COMMAND chmod a+x "${PROJECT_SOURCE_DIR}/proto/build.sh")
execute_process( COMMAND bash "-c" "${PROJECT_SOURCE_DIR}/proto/build.sh")

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(can_driver
    can_ros_node.cc
    can_ros.cc
    can_manager.cc
    can_interface_base/can_reader_base.cc
    can_interface_base/can_writer_base.cc
    can_readers/dm_motor_reader.cc
    can_writers/dm_motor_writer.cc
    common/file.cc
    proto/can_driver_config.pb.cc
)

target_link_libraries(can_driver PUBLIC
    ${catkin_LIBRARIES} ${Protobuf_LIBRARIES} glog
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# install executables ans libraries
install(TARGETS can_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install launch and config
install(DIRECTORY launch conf scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)