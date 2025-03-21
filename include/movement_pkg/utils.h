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

// ===== ANSI COLOR CODES =====
#define DEFAULT_TEX RESET_TEXT  "\033[0m"

#define RED_TEXT                "\033[31m"
#define BOLD_RED_TEXT           "\033[1;31m"

#define GREEN_TEXT              "\033[32m"
#define BOLD_GREEN_TEXT         "\033[1;32m"

#define YELLOW_TEXT             "\033[33m"
#define BOLD_YELLOW_TEXT        "\033[1;33m"

#define PINK_TEXT               "\033[35m"
#define BOLD_PINK_TEXT          "\033[1;35m"

#define CYAN_TEXT               "\033[36m"
#define BOLD_CYAN_TEXT          "\033[1;36m"

// ===== Filename utility =====
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Macro to resolve color name + bold
#define COLOR_TEXT(name, bold) (bold ? BOLD_##name##_TEXT : name##_TEXT)

// ===== SUCCESS LOG (stream-based, green, bold option) =====
#define ROS_SUCCESS_LOG(msg) \
    ROS_INFO_STREAM_COND(DEBUG_PRINT, BOLD_GREEN_TEXT << "[SUCCESS] " << msg << RESET_TEXT)

// ===== ERROR LOG with DEBUG_PRINT check and colored [filename:line] =====
#define ROS_ERROR_LOG(msg, bold, ...) \
    ROS_ERROR_COND(DEBUG_PRINT, "%s[%s:%d] " RESET_TEXT msg, (bold ? BOLD_PINK_TEXT : PINK_TEXT), __FILENAME__, __LINE__, ##__VA_ARGS__)

// Flexible color info log
#define ROS_COLORED_INFO_LOG(msg, color_name, bold, ...) \
    ROS_INFO_COND(DEBUG_PRINT, "%s[%s:%d] " RESET_TEXT "%s" msg RESET_TEXT, \
    (bold ? BOLD_PINK_TEXT : PINK_TEXT), __FILENAME__, __LINE__, COLOR_TEXT(color_name, bold), ##__VA_ARGS__)


class utils 
{
protected:
    utils();

    ros::NodeHandle nh;  // Shared NodeHandle
    int robot_id;
    bool DEBUG_PRINT = true;

    void setModule(const std::string& module_name);
    void goAction(int page);
    std::string getDataFilePath(const std::string& filename) ;
    std::vector<std::vector<float>> loadPositions();

    // ROS services
    ros::ServiceClient set_joint_module_client;
    ros::ServiceClient get_joint_module_client;

    // ROS LOG ONCE
    void ROS_TAGGED_ONCE_LOG(const std::string& msg,
        const std::string& color_name = "DEFAULT",
        bool bold = false,
        const std::string& tag = "");


public:
    virtual ~utils() = default;

private:

    // ROS variables
    ros::Publisher action_pose_pub_;
    std_msgs::Int32 action_msg_;

    // ROS LOG ONCE
    std::unordered_map<std::string, bool> already_logged_tags_;
    
    //standing up txt
    const int rows_ = 40;
    const int cols_ = 6;
};

#endif  // UTILS_H
