cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_traj_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/turtlebot_traj_ctrl_node.cpp
  src/turtlebot_traj_ctrl.cpp
  src/traj_controller.cpp
  src/fl_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)