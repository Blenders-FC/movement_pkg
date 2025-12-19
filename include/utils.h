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
#include "robotis_controller_msgs/srv/SetModule.h"
#include "robotis_controller_msgs/srv/GetJointModule.h"
#include <robotis_controller_msgs/msg/StatusMsg.h>
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
protected:
    utils()
    {
        node_ = rclcpp::Node::make_shared("utils");
        robot_id = 1;
        DEBUG_PRINT = true;

        // Publisher
        action_pose_pub_ = node_->create_publisher<std_msgs::msg::Int32>("action", 10);

        // Service clients
        set_joint_module_client_ =
           node_->create_client<robotis_controller_msgs::srv::SetModule>("set_joint_module");
        get_joint_module_client_ =
           node_->create_client<robotis_controller_msgs::srv::GetJointModule>("get_joint_module");
    }

    void setModule(const std::string& module)
    {
        auto req = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
        req->module_name = module;
        if (!set_joint_module_client_->wait_for_service(1s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Service unavailable");
            return;
        }
        set_joint_module_client_->async_send_request(req);
    }

    void goAction(int page)
    {
        std_msgs::msg::Int32 msg;
        msg.data = page;
        action_pose_pub_->publish(msg);
    }

protected:
    rclcpp::Node::SharedPtr node_;
    int robot_id;
    bool DEBUG_PRINT;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr action_pose_pub_;
    rclcpp::Client<robotis_controller_msgs::srv::SetModule>::SharedPtr set_joint_module_client_;
    rclcpp::Client<robotis_controller_msgs::srv::GetJointModule>::SharedPtr get_joint_module_client_;
};

