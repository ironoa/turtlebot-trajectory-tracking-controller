#ifndef TEST_SIMPLEH_
#define TEST_SIMPLE_H_

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>

class test_simple
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Publisher vehicleCommand_publisher;

    /* Parameters from ROS parameter server */
    double run_period, V0, V1, omega0, omega1, T01;

    /* Node periodic task */
    void PeriodicTask(void);

    /* Node state variables */
    double linear_velocity, angular_velocity;

  public:
    double RunPeriod;

    void Prepare(void);
    
    void RunPeriodically();
    
    void Shutdown(void);

};

#endif /* TEST_SIMPLE_H_ */
