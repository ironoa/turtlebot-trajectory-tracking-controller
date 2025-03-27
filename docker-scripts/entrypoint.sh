#bin/bash


echo "checking software versions..."
echo "ROS: ${ROS_DISTRO}"
echo "done"

# source /colcon_ws/install/setup.bash
source /catkin_ws/devel/setup.bash

/bin/bash


# ######### useful commands when you are in ################
# catkin_make

# installed xquartz on mac: https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285?permalink_comment_id=3477013#gistcomment-3477013
# xhost + 127.0.0.1
# xhost + localhost

# start the master
# roscore

# Turtlesim
# rosrun turtlesim turtlesim_node
# rosrun turtlesim turtle_teleop_key

# DEBUG
# rostopic list
# rosnode list
# rosnode info /turtlesim
# rostopic echo /turtle1/cmd_vel
# rostopic pub /turtle1/cmd_vel geometry_msgs/Twist ...

# Visualization
# rqt_graph

# create package: https://wiki.ros.org/ROS/Tutorials/CreatingPackage
# catkin_create_pkg my_first_package std_msgs rospy roscpp

# rosrun hello_world node_example
# rostopic echo /topic2
# rostopic pub /topic1 ...
# roslaunch hello_world node_example.launch

# Homework1 #######################
# catkin_create_pkg homework_1 std_msgs roscpp

# roslaunch homework_1 counter.launch

# ci 93 => -15
# cf 43 => 5
################


### Odeint ####
# catkin_create_pkg ode_simulator roscpp std_msgs
# rqt_plot /system_output/data[1] /system_output/data[2] /system_input/data /setpoint/data
# rosbag record -a –duration=25
# rosbag record -a -O /catkin_ws/src/turtlebot_traj_ctrl/trajectory.bag --duration=25