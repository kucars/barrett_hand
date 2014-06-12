cmake_minimum_required(VERSION 2.8.3)
project(barrett_hand_control)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages




include_directories(include ${catkin_INCLUDE_DIRS})

add_library(barrett_hand_controllers
    src/joint_velocity_controller_tactile.cpp
  )

target_link_libraries(barrett_hand_controllers ${catkin_LIBRARIES})
#add_executable(barrett_interface src/BarrettInterface.cpp)
#target_link_libraries(
#  barrett_interface
#  ${catkin_LIBRARIES}
#)

#add_executable(move_hand_action_server src/MoveHandActionServer.cpp)

#target_link_libraries(
#  move_hand_action_server
#  ${catkin_LIBRARIES}
#)
