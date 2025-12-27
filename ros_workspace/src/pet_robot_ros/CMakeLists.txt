cmake_minimum_required(VERSION 3.0.2)
project(pet_robot_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  message_generation
)

## Declare ROS messages and services
add_message_files(
  FILES
  Emotion.msg
  MoodLog.msg
  RobotState.msg
  SensorData.msg
)

add_service_files(
  FILES
  SetEmotion.srv
  GetAffirmation.srv
  LogMood.srv
)

add_action_files(
  FILES
  BreathingExercise.action
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    message_runtime
)

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
## Install Python scripts
catkin_install_python(PROGRAMS
  nodes/robot_controller_node.py
  nodes/mental_health_node.py
  nodes/emotion_node.py
  # REMOVED nodes/motor_control_node.py because it is missing from the folder
  scripts/test_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
