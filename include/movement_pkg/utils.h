/*
    Authors:
        Pedro Deniz
*/

#ifndef UTILS_H
#define UTILS_H

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>

#include "robotis_controller_msgs/SetModule.h"



class utils {
public:
    static utils& getInstance() {
        static utils instance;  // Singleton instance - Ensures only one instance of utils class exists in mem
        return instance;
    }

    ros::NodeHandle nh;  // Shared NodeHandle
    int robot_id;

    void setModule(const std::string& module_name);
    void goAction(int page);

private:

    ros::ServiceClient set_joint_module_client;

    // Constructor is private so no other code can create an utils instance
    utils() : nh(ros::this_node::getName()) {  // Private constructor with NodeHandle
        nh.param<int>("robot_id", robot_id, 0);
        ROS_INFO("Loaded utils: robot_id=%d", robot_id);
        
        set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");
    }

    // Ensuring all nodes use the same instance
    utils(const utils&) = delete;  // Prevent copying
    void operator=(const utils&) = delete;
};

#endif  // UTILS_H
