#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "turtlebot_traj_ctrl/fl_controller.h"
#include "turtlebot_traj_ctrl/traj_controller.h"

class turtlebot_traj_ctrl
{ 
private:
    ros::NodeHandle Handle;
    ros::Subscriber state_subscriber;
    ros::Publisher control_publisher;
    ros::Publisher trajectory_publisher;
        
    double P_dist, Ts;;
    double T,alfa, K_prop_x, K_prop_y, T_integral_x, T_integral_y;

    FLC* fl_controller;
    TJC* traj_controller;
    
    void state_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void PeriodicTask(void);

public:
    void Prepare(void);
    void RunPeriodically(void);
    void Shutdown(void);
};