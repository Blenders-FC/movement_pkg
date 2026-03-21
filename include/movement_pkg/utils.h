/*
    Authors:
        Pedro Deniz

        Ricardo Berumen
*/

#ifndef UTILS_H
#define UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <random>

#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <robotis_controller_msgs/msg/status_msg.hpp>

#include "robotis_controller_msgs/srv/get_joint_module.hpp"
#include "robotis_controller_msgs/srv/set_joint_module.hpp"
#include "robotis_controller_msgs/srv/set_module.hpp"
#include "robotis_controller_msgs/srv/load_offset.hpp"
#include <behaviortree_cpp/blackboard.h>
//#include "movement_pkg/blackboard.h"


// ===== ANSI COLOR CODES =====
#define RED_TEXT            "\033[91m"
#define BOLD_RED_TEXT       "\033[1;91m"
#define RED_BG_TEXT         "\033[41m"

#define GREEN_TEXT          "\033[92m"
#define BOLD_GREEN_TEXT     "\033[1;92m"
#define GREEN_BG_TEXT       "\033[42m"

#define YELLOW_TEXT         "\033[93m"
#define BOLD_YELLOW_TEXT    "\033[1;93m"
#define YELLOW_BG_TEXT      "\033[43m"

#define BLUE_TEXT           "\033[34m"
#define BOLD_BLUE_TEXT      "\033[1;34m"
#define BLUE_BG_TEXT        "\033[44m"

#define MAGENTA_TEXT        "\033[95m"
#define BOLD_MAGENTA_TEXT   "\033[1;95m"
#define MAGENTA_BG_TEXT     "\033[45m"

#define CYAN_TEXT           "\033[96m"
#define BOLD_CYAN_TEXT      "\033[1;96m"
#define CYAN_BG_TEXT        "\033[46m"

#define ORANGE_TEXT         "\033[38;5;208m"
#define BOLD_ORANGE_TEXT    "\033[1;38;5;208m"
#define ORANGE_BG_TEXT      "\033[48;5;208m"

#define VIOLET_TEXT         "\033[38;5;129m"
#define BOLD_VIOLET_TEXT    "\033[1;38;5;129m"
#define VIOLET_BG_TEXT      "\033[48;5;129m"

#define PINK_TEXT           "\033[38;5;213m"
#define BOLD_PINK_TEXT      "\033[1;38;5;213m"

#define TEAL_TEXT           "\033[38;5;30m"
#define BOLD_TEAL_TEXT      "\033[1;38;5;30m"
#define TEAL_BG_TEXT        "\033[48;5;30m"

#define BLACK_BG_TEXT       "\033[40m"
#define GRAY_BG_TEXT        "\033[100m"
#define BROWN_BG_TEXT       "\033[48;5;94m"
#define DEFAULT_TEXT        "\033[0m"

// ===== Filename utility =====
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Helper macros to resolve color name + bold
#define _COLOR_TEXT(name) name##_TEXT
#define _BOLD_COLOR_TEXT(name) BOLD_##name##_TEXT
#define COLOR_TEXT(name, bold) ((bold) ? _BOLD_COLOR_TEXT(name) : _COLOR_TEXT(name))


class utils
{
public:
    explicit utils(rclcpp::Node::SharedPtr node);

    virtual ~utils() = default;

    //Blackboard* getBlackboard() { return &blackboard; }

    void setModule(const std::string& module_name);
    std::string getModule(const std::string& joint_name);
    void goAction(int page);

    std::string getDataFilePath(const std::string& filename);
    std::vector<std::vector<float>> loadPositions();

    static void resetLoggedTags();

protected:  
    utils();
    rclcpp::Node::SharedPtr node_;
    int robot_id;
    bool DEBUG_PRINT = true;
    //Blackboard blackboard;
    template <typename T>
        T clamp(T val, T min_val, T max_val)
        {
            return std::max(min_val, std::min(val, max_val));
        }

private:
    // Service clients
    rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_joint_module_client_;
    rclcpp::Client<robotis_controller_msgs::srv::GetJointModule>::SharedPtr get_joint_module_client_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr action_pose_pub_;    
    static std::unordered_map<std::string, bool> already_logged_tags_;
    static std::unordered_map<std::string, std::pair<const char*, const char*>> color_map;
    
    //standing up txt
    const int rows_ = 40;
    const int cols_ = 6;
    
    std::string last_module;
    const char* resolveColor(const std::string& color, bool bold);

};

#endif  // UTILS_H
