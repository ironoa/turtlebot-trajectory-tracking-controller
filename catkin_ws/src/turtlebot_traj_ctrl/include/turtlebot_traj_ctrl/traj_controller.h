#ifndef PLANNER_H
#define PLANNER_H

#include <std_msgs/Float64MultiArray.h>

class TJC {
public:
    TJC(double P_dist, double omega_0, double alfa, double K_prop_x, double K_prop_y, double T_integral_x, double T_integral_y, double Ts);
    void getTrajectory(double &vx, double &vy);
    void getReference(double &xref, double &yref, double &xPref, double &yPref);
    void getTrackingError(double &t, double &tracking_error_x, double &tracking_error_y);
    void execute();
    void setMeasurement(const std_msgs::Float64MultiArray::ConstPtr& msg);

private:
    double P_dist, omega_0, alfa, K_prop_x, K_prop_y, T_integral_x, T_integral_y, Ts; 
    double x, y, theta; //measured
    double vPx_integral_prev, vPy_integral_prev; //state
    double vPx, vPy; //output
    double xref, yref, xPref, yPref; //reference to be displayed
    double tracking_error_x, tracking_error_y; //tobe displayed

    void initialize();
    void state_to_P(double &xP, double &yP);
    void reference_to_P(double xref,double yref, double &xPref, double &yPref);
    double computePI(double y, double y_setpoint, double K_prop, double T_integral, double &u_integral_prev, double &tracking_error);
};

#endif