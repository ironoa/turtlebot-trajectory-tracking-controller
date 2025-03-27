#include "turtlebot_traj_ctrl/turtlebot_traj_ctrl.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_traj_ctrl");
    
    turtlebot_traj_ctrl node;
    node.Prepare();
    node.RunPeriodically();
    node.Shutdown();
    
    return 0;
}