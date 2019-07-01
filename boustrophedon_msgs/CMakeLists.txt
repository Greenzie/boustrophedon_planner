cmake_minimum_required(VERSION 2.8.3)
project(boustrophedon_msgs)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  genmsg
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
  )

add_message_files(
  DIRECTORY msg
  FILES
  StripingPoint.msg
  StripingPlan.msg
)

add_service_files(
  FILES
  ConvertPlanToPath.srv
)

add_action_files(
  DIRECTORY action
  FILES
  PlanMowingPath.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  nav_msgs
)

catkin_package()
