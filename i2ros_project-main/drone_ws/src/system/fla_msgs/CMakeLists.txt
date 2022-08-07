cmake_minimum_required(VERSION 2.8.3)
project(fla_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FlightStateTransition.msg
  FlightEvent.msg
  FlightCommand.msg
  JoyDef.msg
  ControlMessage.msg
  NodeList.msg
  NodeStatus.msg
  ProcessStatus.msg
  Box.msg
  WaypointList.msg
  TelemString.msg
  Detection.msg
  ImageDetections.msg
  Latency.msg
  ImageSegmentation.msg
  Keypoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

