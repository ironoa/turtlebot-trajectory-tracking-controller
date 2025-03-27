#include "turtlebot_traj_ctrl/turtlebot_traj_ctrl.h"
#include "turtlebot_traj_ctrl/utils.h"

void turtlebot_traj_ctrl::Prepare(void) {
    // Load parameters
    loadParam("/controller/P_dist", P_dist);
    loadParam("/controller/Ts", Ts);

    loadParam("/trajectory_controller/T", T);
    loadParam("/trajectory_controller/alfa", alfa);
    loadParam("/trajectory_controller/K_prop_x", K_prop_x);
    loadParam("/trajectory_controller/K_prop_y", K_prop_y);
    loadParam("/trajectory_controller/T_integral_x", T_integral_x);
    loadParam("/trajectory_controller/T_integral_y", T_integral_y);


    // Initialize feedback linearized controller
    fl_controller = new FLC(P_dist);

    // Initialize trajectory controller
    traj_controller = new TJC(P_dist, 2*M_PI/T, alfa, K_prop_x, K_prop_y, T_integral_x, T_integral_y, Ts);

    // Setup ROS topics
    state_subscriber = Handle.subscribe("state", 1, &turtlebot_traj_ctrl::state_MessageCallback, this);
    control_publisher = Handle.advertise<std_msgs::Float64MultiArray>("cmd", 1);
    trajectory_publisher = Handle.advertise<std_msgs::Float64MultiArray>("trajectory", 1);
}

void turtlebot_traj_ctrl::state_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {    
    fl_controller->setMeasurement(msg);
    traj_controller->setMeasurement(msg);
}

void turtlebot_traj_ctrl::PeriodicTask(void)
{
    double time = ros::Time::now().toSec();

    double vpx, vpy;
    traj_controller->execute();
    traj_controller->getTrajectory(vpx, vpy);

    fl_controller->setReference(vpx, vpy);
    fl_controller->execute();

    double v, omega; 
    fl_controller->getControl(v, omega);
    std_msgs::Float64MultiArray cmd_msg;
    cmd_msg.data = {time, v, omega};
    control_publisher.publish(cmd_msg);

    //a figure showing the reference and actual robot trajectory;
    double xref, yref, xPref, yPref, tracking_error_x, tracking_error_y, tracking_error_time;
    traj_controller->getReference(xref, yref, xPref, yPref);
    traj_controller->getTrackingError(tracking_error_time,tracking_error_x, tracking_error_y);
    std_msgs::Float64MultiArray trj_msg;
    trj_msg.data = {xref, yref, xPref, yPref, tracking_error_time, tracking_error_x, tracking_error_y, time};
    ROS_INFO_STREAM("CONTROLLER | Trajectory and stats -> time: " << tracking_error_time << " | xref: " << xref << " | yref: " << yref << " | xPref: " << xPref << " | yPref: " << yPref << " | tracking_error_x: " << tracking_error_x << " | tracking_error_y: " << tracking_error_y);
    trajectory_publisher.publish(trj_msg);
}

void turtlebot_traj_ctrl::RunPeriodically(void) {

    double period = Ts;
    ros::Rate loop_rate(1.0/period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), period, 1.0/period);

    while (ros::ok())
    {
        ros::spinOnce();  // Handle callbacks first
        PeriodicTask();   // Then execute control task
        loop_rate.sleep();
    }
}

void turtlebot_traj_ctrl::Shutdown(void) {
    delete fl_controller;
    delete traj_controller;
}