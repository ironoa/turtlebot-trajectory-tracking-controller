#ifndef TURTLEBOT_SIMULATOR_H
#define TURTLEBOT_SIMULATOR_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include <rosgraph_msgs/Clock.h>
#include "engine/turtlebot_simulator_ode.h"
#include <fstream>

class turtlebot_simulator
{
private:
    ros::NodeHandle Handle;
    ros::Subscriber input_subscriber;
    ros::Publisher output_publisher, clock_publisher;
    
    int num_steps;
    double sim_speed;
    double dt;

    // Initial state for unicycle
    double x0, y0, theta0;

    double Ta; // time constant of the low-level velocity controllers
    
    void input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void PeriodicTask(void);
    
    turtlebot_simulator_ode* simulator;

    std::ofstream csv_file_;

public:
    void Prepare(void);
    void RunPeriodically(void);
    void Shutdown(void);
};

#endif