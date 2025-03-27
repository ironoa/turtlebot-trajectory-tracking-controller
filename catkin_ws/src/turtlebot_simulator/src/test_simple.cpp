#include "turtlebot_simulator/test_simple.h"
#include "turtlebot_simulator/utils.h"

void test_simple::Prepare(void)
{
    // Get parameters
    loadParam("/test/run_period", run_period);
    loadParam("/test/T01", T01);
    loadParam("/test/V0", V0);
    loadParam("/test/V1", V1);
    loadParam("/test/omega0", omega0);
    loadParam("/test/omega1", omega1);

    /* ROS topics */
    vehicleCommand_publisher = Handle.advertise<std_msgs::Float64MultiArray>("cmd", 1);

    /* Initialize node state */
    linear_velocity = angular_velocity = 0.0;

    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void test_simple::RunPeriodically()
{
    ros::Rate LoopRate(1.0/run_period);

    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), run_period, 1.0/run_period);

    while (ros::ok())
    {
        PeriodicTask();
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void test_simple::Shutdown(void)
{
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void test_simple::PeriodicTask(void)
{
    /* Vehicle commands */
    if (ros::Time::now().toSec()<=T01)
    {
        linear_velocity  = V0;
        angular_velocity = omega0;
    }
    else
    {
        linear_velocity  = V1;
        angular_velocity = omega1;
    }

    /* Publishing vehicle commands (t, msg->data[0]; velocity, msg->data[1]; steer, msg->data[2]) */
    std_msgs::Float64MultiArray msg;
    msg.data = {ros::Time::now().toSec(), linear_velocity, angular_velocity};
    vehicleCommand_publisher.publish(msg);
}
