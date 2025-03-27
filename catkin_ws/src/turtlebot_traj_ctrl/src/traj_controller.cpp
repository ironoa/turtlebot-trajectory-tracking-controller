#include "turtlebot_traj_ctrl/traj_controller.h"
#include <ros/ros.h> 

TJC::TJC(double P_dist, double omega_0, double alfa, double K_prop_x, double K_prop_y, double T_integral_x, double T_integral_y, double Ts) {
    this -> omega_0 = omega_0;
    this -> alfa = alfa;
    this -> P_dist = P_dist;
    this -> K_prop_x = K_prop_x;
    this -> K_prop_y = K_prop_y;
    this -> T_integral_x = T_integral_x;
    this -> T_integral_y = T_integral_y;
    this -> Ts = Ts;
    
    initialize();
}

void TJC::initialize() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    vPx_integral_prev = 0.0;
    vPy_integral_prev = 0.0;

    xref = yref = xPref = yPref = 0.0;

    tracking_error_x = 0.0;
    tracking_error_y = 0.0;
}

void TJC::setMeasurement(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    x = msg->data[1];
    y = msg->data[2];
    theta = msg->data[3];
}

void TJC::state_to_P(double &xP, double &yP) {
    xP = x + P_dist*cos(theta);
    yP = y + P_dist*sin(theta);
}

void TJC::reference_to_P(double xref,double yref, double &xPref, double &yPref) {
    xPref = xref + P_dist*cos(theta); 
    yPref = yref + P_dist*sin(theta);
}


double TJC::computePI(double y, double y_setpoint, double K_prop, double T_integral, double &u_integral_prev, double &tracking_error) {

    double error = y_setpoint - y;
    tracking_error = error;

    double proportional = K_prop * error;
    
    double K_integral = 0.0; 
    if(T_integral > 0){
        K_integral = K_prop * Ts/T_integral;
    }
    double integral = u_integral_prev + K_integral * error;

    // Update previous states
    u_integral_prev = integral;

    return proportional + integral;
}


void TJC::execute(void)
{
    // Get current time
    ros::Time current_time = ros::Time::now();    
    double t = current_time.toSec();
    // ROS_INFO_STREAM("CURRENT TIME from simulator: " << t);

    // Trajectory 8
    double xref  = alfa*sin(omega_0*t);
    double vxref = alfa*omega_0*cos(omega_0*t);
    double yref  = alfa*sin(omega_0*t)*cos(omega_0*t);
    double vyref = alfa*omega_0*(pow(cos(omega_0*t),2.0)-pow(sin(omega_0*t),2.0));

    // ROS_INFO_STREAM("TRAJECTORY GENERATION: t: " << t << " xref: " << xref << " yref: " << yref);

    // Transform trajectory to point P
    double xPref, yPref, xP, yP;
    TJC::reference_to_P(xref, yref, xPref, yPref);
    TJC::state_to_P(xP, yP);

    //save reference
    this->xPref = xPref;
    this->yPref = yPref;
    this->xref = xref;
    this->yref = yref;

 
    // Trajectory tracking law with a PI, plus velocity forward
    vPx = vxref + TJC::computePI(xP, xPref, K_prop_x, T_integral_x, vPx_integral_prev, tracking_error_x);
    vPy = vyref + TJC::computePI(yP, yPref, K_prop_y, T_integral_y, vPy_integral_prev, tracking_error_y);
}

void TJC::getTrajectory(double &vx, double &vy)
{
    vx = this->vPx;
    vy = this->vPy;
}

void TJC::getReference(double &xref, double &yref, double &xPref, double &yPref)
{
    xref = this->xref;
    yref = this->yref;
    xPref = this->xPref;
    yPref = this->yPref;
}

void TJC::getTrackingError(double &t, double &tracking_error_x, double &tracking_error_y)
{
    t = ros::Time::now().toSec(); 
    tracking_error_x = this->tracking_error_x;
    tracking_error_y = this->tracking_error_y;
}
