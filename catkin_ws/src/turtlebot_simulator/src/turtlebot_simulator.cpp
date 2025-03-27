#include "turtlebot_simulator/turtlebot_simulator.h"
#include "turtlebot_simulator/utils.h"
#include <ros/package.h>

void turtlebot_simulator::Prepare(void)
{
    // Get parameters
    loadParam("/simulator/sim_speed", sim_speed);
    loadParam("/simulator/dt", dt);
    loadParam("/simulator/num_steps", num_steps);

    loadParam("/simulator/Ta", Ta);
    
    // Get initial state
    loadParam("/simulator/initial_conditions/x0", x0);
    loadParam("/simulator/initial_conditions/y0", y0);
    loadParam("/simulator/initial_conditions/theta0", theta0);


    // Setup simulator
    simulator = new turtlebot_simulator_ode(dt);
    simulator->setInitialState(x0, y0, theta0);

    // ROS communication
    input_subscriber = Handle.subscribe("cmd", 1, &turtlebot_simulator::input_MessageCallback, this);
    output_publisher = Handle.advertise<std_msgs::Float64MultiArray>("state", 1);
    clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("clock", 1);
}

void turtlebot_simulator::input_MessageCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 3) {
        ROS_ERROR("Expected input array of size 3 [time, v, omega]");
        return;
    }
    
    // Set input values [v, omega]
    simulator->setInputValues(msg->data[1], msg->data[2]);

    ROS_INFO_STREAM("SIMULATOR | Received Command -> time: " << msg->data[0] << " | v: " << msg->data[1] << " | omega: " << msg->data[2]);
}

void turtlebot_simulator::PeriodicTask(void)
{
    /* Integrate one step ahead */
    simulator->integrate();

    // Get current state and time
    double x, y, theta, time, linvelocity_act, angvelocity_act;
    simulator->getState(x, y, theta);
    simulator->getVelocities(linvelocity_act, angvelocity_act);
    simulator->getTime(time);

    ROS_INFO_STREAM("SIMULATOR | New State -> t: " << time << " | x: " << x << " | y: " << y << " | theta: " << theta << " | linvelocity_act: " << linvelocity_act << " | angvelocity_act: " << angvelocity_act);

    /* Print simulation time every 5 sec */
    // if (std::fabs(std::fmod(time,5.0)) < 1.0e-3)
    //     ROS_INFO("Simulator time: %d seconds", (int) time);

    // Publish state
    std_msgs::Float64MultiArray state_msg;
    state_msg.data = {time, x, y, theta, linvelocity_act, angvelocity_act};
    output_publisher.publish(state_msg);

    /* Publish clock: Publish simulation time */
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(time);
    clock_publisher.publish(clockMsg);
}

void turtlebot_simulator::RunPeriodically(void)
{
    ROS_INFO("Node %s running.", ros::this_node::getName().c_str());

    // Wait other nodes start
    sleep(1.0);

    for (int k = 0; k <= num_steps; k++) 
    {
        if (!ros::ok()) break;
        
        // Step simulation
        PeriodicTask();
        
        // Handle callbacks
        ros::spinOnce();

        usleep((dt/sim_speed) * 1000000);  
    }

    ROS_INFO("Simulation completed after %d steps", num_steps);
    ros::shutdown();
}

void turtlebot_simulator::Shutdown(void)
{
    if(simulator != nullptr) {
        delete simulator;
    }
    ROS_INFO("Node shutdown complete");
}