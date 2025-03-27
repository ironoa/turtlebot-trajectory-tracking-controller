#include "engine/turtlebot_simulator_ode.h"
#include <cmath>
#include <ros/ros.h>

turtlebot_simulator_ode::turtlebot_simulator_ode(double deltaT)
{
    // Initialize time and dt
    t = 0.0;
    dt = deltaT;

    // Initialize flags (no model parameters tobe set)
    modelParams_set = true;

    // Initial state values (x, y, theta)
    state.resize(3);
    state[0] = 0.0;  // x position
    state[1] = 0.0;  // y position
    state[2] = 0.0;  // theta orientation

    v = 0.0;
    omega = 0.0;
}

void turtlebot_simulator_ode::setInitialState(double x, double y, double theta)
{
    // Check model parameters are set
    if (!modelParams_set) {
        throw std::invalid_argument("Model parameters not set!");
    }

    // Initial state values
    state[0] = x;      // x position
    state[1] = y;      // y position
    state[2] = theta;  // orientation
}

void turtlebot_simulator_ode::simulate_motors(double v, double omega)
{
    // Simulate the motors and the low-level velocity controllers;

    this->v = dt/(Ta+dt) * v + Ta/(Ta+dt) * this->v;
    this->omega = dt/(Ta+dt) * omega + Ta/(Ta+dt) * this->omega;
}

void turtlebot_simulator_ode::setInputValues(double v, double omega)
{
    //extended Model !
    //simulate the motors and the low-level velocity controllers
    simulate_motors(v, omega);
}

void turtlebot_simulator_ode::getVelocities(double &v, double &omega) 
{ 
    v = this -> v; 
    omega = this -> omega; 
};

void turtlebot_simulator_ode::integrate()
{
    // Check model parameters are set
    if (!modelParams_set) {
        throw std::invalid_argument("Model parameters not set!");
    }

    // ROS_INFO_STREAM("OLD STATE FOR xytheta: time: " << t << " | x: " << state[0] << " | y: " << state[1] << " | theta: " << state[2]);

    // Integrate for one step ahead
    stepper.do_step(std::bind(&turtlebot_simulator_ode::simulator_ode, this, 
                             std::placeholders::_1, std::placeholders::_2, 
                             std::placeholders::_3), state, t, dt);

    // ROS_INFO_STREAM("NEW STATE FOR xytheta: time: " << t << " | x: " << state[0] << " | y: " << state[1] << " | theta: " << state[2]);  

    // Update time
    t += dt;
}

void turtlebot_simulator_ode::simulator_ode(const state_type &state, state_type &dstate, double t)
{
    // Current state
    // const double x = state[0];     // x position
    // const double y = state[1];     // y position
    const double theta = state[2]; // orientation

    // Unicycle kinematic model equations
    dstate[0] = v * std::cos(theta);  // dx/dt = v*cos(theta)
    dstate[1] = v * std::sin(theta);  // dy/dt = v*sin(theta)
    dstate[2] = omega;                // dtheta/dt = omega
}

void turtlebot_simulator_ode::getState(double &x, double &y, double &theta)
{
    x = state[0];
    y = state[1];
    theta = state[2];
}

void turtlebot_simulator_ode::getTime(double &time)
{
    time = t;
}