cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_control)
add_subdirectory(owd)
#add_subdirectory(owd/openwam)
#add_subdirectory(owd/openmath)
include_directories (owd/openwam owd/openmath)
add_definitions("-O0 -ggdb3 -DWRIST -DBH8")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy roscpp actionlib barrett_hand_msgs owd_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(move_hand_action_server src/MoveHandActionServer.cpp)

target_link_libraries(
  move_hand_action_server
  ${catkin_LIBRARIES}
)
