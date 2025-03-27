#include "turtlebot_traj_ctrl/fl_controller.h"

FLC::FLC(double P_dist) {
    
    this -> P_dist = P_dist;

    initialize();
}

void FLC::initialize() {
    theta = 0.0;
    vpx = 0.0;
    vpy = 0.0;
}

void FLC::setMeasurement(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    theta = msg->data[3];
}

void FLC::setReference(double vpx, double vpy) {
    this -> vpx = vpx;
    this -> vpy = vpy;
}

void FLC::execute() {
    // FLC: Feedback Linearization of a Unicycle Robot
    v = vpx * cos(theta) + vpy * sin(theta);
    omega = (vpy * cos(theta) - vpx * sin(theta))/P_dist;
}

void FLC::getControl(double &v, double &omega) {
    v = this->v;
    omega = this->omega;
}