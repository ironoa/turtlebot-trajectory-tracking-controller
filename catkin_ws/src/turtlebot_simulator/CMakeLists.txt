cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib    # Add this line
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs roslib 
)

include_directories(
  include
  include/turtlebot_simulator
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/turtlebot_simulator_node.cpp
  src/turtlebot_simulator.cpp
  src/engine/turtlebot_simulator_ode.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_simple
  src/test_simple_node.cpp
  src/test_simple.cpp
)
target_link_libraries(test_simple
  ${catkin_LIBRARIES}
)
