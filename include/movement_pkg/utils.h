/*
    Authors:
        Pedro Deniz
*/

#ifndef UTILS_H
#define UTILS_H

#define GREEN_TEXT "\033[1;32m"
#define RESET_TEXT "\033[0m"

#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>

#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

#include "robotis_controller_msgs/SetModule.h"
#include <robotis_controller_msgs/StatusMsg.h>


class utils 
{
protected:
    utils();

    ros::NodeHandle nh;  // Shared NodeHandle
    int robot_id;

    void setModule(const std::string& module_name);
    void goAction(int page);
    std::string getDataFilePath(const std::string& filename) ;
    std::vector<std::vector<float>> loadPositions();

    // ROS services
    ros::ServiceClient set_joint_module_client;
    ros::ServiceClient get_joint_module_client;

public:
    virtual ~utils() = default;

private:

    // ROS variables
    ros::Publisher action_pose_pub_;
    std_msgs::Int32 action_msg_;
    
    //standing up txt
    const int rows_ = 40;
    const int cols_ = 6;
};

#endif  // UTILS_H
