Modules: Trajectory Tracking Controller + Feedback Linearizer Controller with point P which is ditant P_dist.
PI is embedded in Trajectory Tracking Controller module

## Requirements
- as per instruction: `projects are verified using ROS Noetic`
- Check Dockerfile to see exactly what packages are required (Included here in the package src just for this purpose)
  - i.e. to run the python scripts: `pip3 install matplotlib control numpy`

## HOW TO RUN PART 2 (SIMULATOR+CONTROLLER)
```sh
catkin_make
roslaunch turtlebot_traj_ctrl turtlebot_traj_ctrl_with_bag.launch # a bag will be recored for 25 (sim_time) seconds -> please stop manually with ctrl+c
# roslaunch turtlebot_traj_ctrl turtlebot_traj_ctrl.launch # if you don't want the bag recording
python3 script/plot_result.py trajectory.bag
```

## HOW TO RUN BODE ANALYSIS
```sh
python3 script/bode_tuning_x.py
# Tune Kp_x and Ti_x in the code
```