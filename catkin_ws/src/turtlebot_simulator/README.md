the simulator takes the desired velocity commands (linear and angular) and integrates the unicycle model to produce the next state.  
In order to simulate the motors and the low-level velocity controllers, the unicycle kinematic model is extended with transfer functions.


## Requirements
- as per instruction: `projects are verified using ROS Noetic`
- Check Dockerfile to see exactly what packages are required (Included here in the package src just for this purpose)

## HOW TO RUN PART 1, with simple test node
```sh
catkin_make
roslaunch turtlebot_simulator turtlebot_simulator_test_with_bag.launch # a bag will be recored for 25 (sim_time) seconds -> please stop manually with ctrl+c
# roslaunch turtlebot_simulator turtlebot_simulator_test.launch # if you don't want the bag recording
python3 script/plot_result.py trajectory.bag
```
