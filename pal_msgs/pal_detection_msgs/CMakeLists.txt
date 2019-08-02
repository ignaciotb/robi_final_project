cmake_minimum_required(VERSION 2.8.3)
project(pal_detection_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    geometry_msgs
    actionlib_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    Detection2d.msg
    Detections2d.msg
    FaceDetection.msg
    FaceDetections.msg
    Gesture.msg
    LegDetections.msg
    PersonDetection.msg
    PersonDetections.msg
    RecognizedActions.msg
    RotatedDetection2d.msg
    TexturedObjectDetection.msg
    WaveDetection.msg
    RecognizedObject.msg
    RecognizedObjectArray.msg
)

add_action_files(
    DIRECTORY action FILES
    RecognizeObjects.action
)

add_service_files(
  DIRECTORY srv
  FILES
    AddTexturedObject.srv
    SelectTexturedObject.srv
    Recognizer.srv
    SetDatabase.srv
    StartEnrollment.srv
    StopEnrollment.srv
    ChangeObjectRecognizerModel.srv
)

generate_messages(DEPENDENCIES sensor_msgs geometry_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs geometry_msgs actionlib_msgs)
