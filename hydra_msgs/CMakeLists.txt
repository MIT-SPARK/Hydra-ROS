cmake_minimum_required(VERSION 3.1)
project(hydra_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES ActiveLayer.msg DsgUpdate.msg)
add_service_files(FILES GetDsg.srv QueryFreespace.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)
