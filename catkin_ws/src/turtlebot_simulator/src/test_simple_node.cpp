#include "turtlebot_simulator/test_simple.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_simple");
  
  test_simple test_simple_node;
  test_simple_node.Prepare();
  test_simple_node.RunPeriodically();
  test_simple_node.Shutdown();
  
  return (0);
}

