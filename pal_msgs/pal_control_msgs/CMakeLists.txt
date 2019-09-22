cmake_minimum_required(VERSION 2.8.3)
project(pal_control_msgs)


find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    actionlib_msgs)

include_directories(include)

add_message_files(
    DIRECTORY msg FILES
    ActuatorCurrentLimit.msg
    OperationalSpaceGoal.msg
    RigidBodyTrajectoryPoint.msg
)
add_action_files(
    DIRECTORY action FILES
    MotionManager.action
    RigidBodyTrajectory.action
)

add_service_files(FILES CurrentLimit.srv)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    std_msgs
    actionlib_msgs)
