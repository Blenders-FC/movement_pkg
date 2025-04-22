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
#include "robotis_controller_msgs/GetJointModule.h"
#include <robotis_controller_msgs/StatusMsg.h>
#include "movement_pkg/blackboard.h"

// ===== ANSI COLOR CODES =====
#define DEFAULT_TEXT        "\033[0m"
#define BOLD_DEFAULT_TEXT   "\033[1;0m"

#define RED_TEXT            "\033[31m"
#define BOLD_RED_TEXT       "\033[1;31m"

#define GREEN_TEXT          "\033[32m"
#define BOLD_GREEN_TEXT     "\033[1;32m"

#define YELLOW_TEXT         "\033[33m"
#define BOLD_YELLOW_TEXT    "\033[1;33m"

#define PINK_TEXT           "\033[35m"
#define BOLD_PINK_TEXT      "\033[1;35m"

#define CYAN_TEXT           "\033[36m"
#define BOLD_CYAN_TEXT      "\033[1;36m"

// ===== Filename utility =====
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Helper macros to resolve color name + bold
#define _COLOR_TEXT(name) name##_TEXT
#define _BOLD_COLOR_TEXT(name) BOLD_##name##_TEXT
#define COLOR_TEXT(name, bold) ((bold) ? _BOLD_COLOR_TEXT(name) : _COLOR_TEXT(name))

// ===== SUCCESS LOG (stream-based, green, bold option) =====
#define ROS_SUCCESS_LOG(msg) \
    ROS_INFO_STREAM_COND(DEBUG_PRINT, BOLD_GREEN_TEXT << "[SUCCESS] " << msg << DEFAULT_TEXT)

// ===== ERROR LOG with DEBUG_PRINT check and colored [filename:line] =====
#define ROS_ERROR_LOG(msg, bold, ...) \
    ROS_ERROR_COND(DEBUG_PRINT, "%s[%s:%d] " msg DEFAULT_TEXT, \
    (bold ? BOLD_PINK_TEXT : PINK_TEXT), __FILENAME__, __LINE__, ##__VA_ARGS__)

// ===== COLORED LOG with DEBUG_PRINT check and colored [filename:line] =====
#define ROS_COLORED_LOG(msg, color_name, bold, ...) \
    ROS_INFO_COND(DEBUG_PRINT, "%s[%s] %s" msg DEFAULT_TEXT, \
    (bold ? BOLD_PINK_TEXT : PINK_TEXT), __FILENAME__, COLOR_TEXT(color_name, bold), ##__VA_ARGS__)

// ===== COLORED ONCE LOG with DEBUG_PRINT check and colored [filename:line] =====
#define ROS_COLORED_ONCE_LOG(msg, color_name, bold, ...) \
    ROS_INFO_COND(DEBUG_PRINT, "%s[%s] %s" msg DEFAULT_TEXT, \
    (bold ? BOLD_PINK_TEXT : PINK_TEXT), __FILENAME__, color_name, ##__VA_ARGS__)


class utils 
{
    protected:
        utils();

        ros::NodeHandle nh;  // Shared NodeHandle
        int robot_id;
        bool DEBUG_PRINT = true;

        void setModule(const std::string& module_name);
        void goAction(int page);
        std::string getDataFilePath(const std::string& filename);
        std::string getModule(const std::string& joint_name);
        std::vector<std::vector<float>> loadPositions();

        // ROS LOG ONCE
        void ROS_TAGGED_ONCE_LOG(const std::string& msg,
            const std::string& color_name = "DEFAULT",
            bool bold = false,
            const std::string& tag = "");

        Blackboard blackboard;


        template <typename T>
        T clamp(T val, T min_val, T max_val)
        {
            return std::max(min_val, std::min(val, max_val));
        }

    public:
        virtual ~utils() = default;
        Blackboard* getBlackboard() { return &blackboard; }

    private:
        // ROS services
        ros::ServiceClient set_joint_module_client;
        ros::ServiceClient get_joint_module_client;

        // color function
        const char* resolveColor(const std::string& color, bool bold);

        // ROS variables
        ros::Publisher action_pose_pub_;
        std_msgs::Int32 action_msg_;

        // ROS LOG ONCE
        std::unordered_map<std::string, bool> already_logged_tags_;
        
        //standing up txt
        const int rows_ = 40;
        const int cols_ = 6;
        
        static std::unordered_map<std::string, std::pair<const char*, const char*>> color_map;
        std::string last_module;
    };

#endif  // UTILS_H
