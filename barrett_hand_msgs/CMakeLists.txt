cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_action_files(
  DIRECTORY action
  FILES MoveHand.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


