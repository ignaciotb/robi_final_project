cmake_minimum_required(VERSION 2.8.3)
project(pal_device_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs actionlib_msgs)

add_message_files(
  FILES
    BatteryState.msg
    Bumper.msg
    LedGroup.msg
    LedEffectParams.msg
    LedFixedColorParams.msg
    LedRainbowParams.msg
    LedFadeParams.msg
    LedBlinkParams.msg
    LedProgressParams.msg
    LedFlowParams.msg
    LedDataArrayParams.msg
    LedEffectViaTopicParams.msg
    LedPreProgrammedParams.msg
)

add_service_files(
    DIRECTORY srv
    FILES
        CancelEffect.srv
        TimedBlinkEffect.srv
        TimedColourEffect.srv
        TimedFadeEffect.srv
)

add_action_files(
    FILES DoTimedLedEffect.action
    )


generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs)
