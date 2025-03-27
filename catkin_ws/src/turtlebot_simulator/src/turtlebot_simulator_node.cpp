#include "turtlebot_simulator/turtlebot_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_simulator");
    
    turtlebot_simulator node;
    node.Prepare();
    node.RunPeriodically();
    node.Shutdown();
    
    return 0;
}