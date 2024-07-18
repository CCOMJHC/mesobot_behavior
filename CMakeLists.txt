cmake_minimum_required(VERSION 3.0.2)
project(mesobot_behavior)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
   FILES
   Behavior.action
#   Action2.action
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mesobot_behavior
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

