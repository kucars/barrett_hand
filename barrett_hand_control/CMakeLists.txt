cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_control)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy roscpp actionlib barrett_hand_msgs owd_msgs libpcan libntcan tf
)

add_subdirectory(owd)
#add_subdirectory(owd/openwam)
#add_subdirectory(owd/openmath)
include_directories (owd/openwam owd/openmath)
add_definitions("-O0 -ggdb3 -DWRIST -DBH8")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages




include_directories(include owd ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(barrett_interface src/BarrettInterface.cpp)
target_link_libraries(
  barrett_interface
  ${catkin_LIBRARIES}
)

#add_executable(move_hand_action_server src/MoveHandActionServer.cpp)

#target_link_libraries(
#  move_hand_action_server
#  ${catkin_LIBRARIES}
#)
