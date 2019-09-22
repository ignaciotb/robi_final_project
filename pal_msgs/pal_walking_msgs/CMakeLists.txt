cmake_minimum_required(VERSION 2.8.3)
project(pal_walking_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation humanoid_nav_msgs std_msgs)

add_message_files(FILES WalkingStatus.msg WalkingStep.msg ContactState.msg SMStatus.msg)

add_service_files(FILES WalkSteps.srv StepOver.srv Sit.srv MoveHipFeet.srv)

generate_messages(DEPENDENCIES geometry_msgs humanoid_nav_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs humanoid_nav_msgs message_runtime)
