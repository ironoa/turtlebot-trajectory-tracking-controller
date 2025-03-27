#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <string>

template<typename T>
void loadParam(const std::string& param_name, T& param_val)
{
    if (!ros::param::get(param_name, param_val))
    {
        ROS_ERROR_STREAM("Could not load parameter: " << param_name);
        throw std::runtime_error("Failed to load required parameter: " + param_name);
    }
    ROS_INFO_STREAM("Loaded " << param_name << ": " << param_val);
}

#endif