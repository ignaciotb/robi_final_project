cmake_minimum_required(VERSION 2.8.3)
project(pal_simulation_msgs)

set(CATKIN_COMPONENTS geometry_msgs message_generation)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

include_directories(include)

add_message_files(DIRECTORY msg FILES ExternalWrench.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS ${CATKIN_COMPONENTS})
