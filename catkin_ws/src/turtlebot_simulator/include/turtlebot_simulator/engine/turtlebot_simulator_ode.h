#ifndef TURTLEBOT_SIMULATOR_ODE_H
#define TURTLEBOT_SIMULATOR_ODE_H

#include <boost/numeric/odeint.hpp>

typedef std::vector<double> state_type;

class turtlebot_simulator_ode
{
    public:

        turtlebot_simulator_ode(double deltaT);
        void setInitialState(double x, double y, double theta);
        void integrate();
        void setInputValues(double v, double omega);
        void getVelocities(double &v, double &omega);
        void getState(double &x, double &y, double &theta);
        void getTime(double &time);

    private:
        // Simulator and integrator variables 
        double t, dt;
        double v;     // linear velocity input
        double omega; // angular velocity input
        double Ta; // time constant of the low-level velocity controllers

        bool modelParams_set;

        state_type state;
        boost::numeric::odeint::runge_kutta_dopri5 < state_type > stepper;

        void simulate_motors(double v, double omega);

        // ODE function
        void simulator_ode(const state_type &state, state_type &dstate, double t);
};

#endif