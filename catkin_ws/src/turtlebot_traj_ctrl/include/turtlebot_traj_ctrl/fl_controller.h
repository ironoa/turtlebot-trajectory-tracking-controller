#ifndef CONTROL_SYSTEM_PID_H
#define CONTROL_SYSTEM_PID_H

#include <std_msgs/Float64MultiArray.h>

class FLC {
public:
    FLC(double P_dist);
    
    void setMeasurement(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void setReference(double vpx, double vpy);
    void getControl(double &v, double &omega);
    void execute();

private:
    double P_dist, theta;
    double vpx, vpy;
    double v, omega;

    void initialize();
};

#endif