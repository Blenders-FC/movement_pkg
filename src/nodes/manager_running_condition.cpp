/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#include "movement_pkg/nodes/manager_running_condition.h"
using namespace std::chrono_literals;

BT::ManagerRunningCondition::ManagerRunningCondition(
    const std::string& name,
    const BT::NodeConfig& config)
: BT::ConditionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("manager_running_condition");
}

BT::NodeStatus BT::ManagerRunningCondition::tick()
{
    // Condition checking and state update
    while (rclcpp::ok())
    {
        rclcpp::sleep_for(1s);

        if (checkManagerRunning(manager_name))
        {
            RCLCPP_INFO(rclcpp::get_logger("ManagerRunningCondition"),
                        "Succeed: connected to manager '%s'",
                        manager_name.c_str());

            already_logged_ = false;  // reset
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            if (!already_logged_)
            {
                RCLCPP_WARN(rclcpp::get_logger("ManagerRunningCondition"),
                            "Waiting for manager '%s' to be available...",
                            manager_name.c_str());
                already_logged_ = true;
            }
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("ManagerRunningCondition"),
                 "ROS2 stopped unexpectedly");

    return BT::NodeStatus::FAILURE;
}

BT::PortsList BT::ManagerRunningCondition::providedPorts()
{
    return {};
}

bool BT::ManagerRunningCondition::checkManagerRunning(std::string& manager_name) {
    // Create a temporary ROS2 node for graph queries
    auto node = rclcpp::Node::make_shared("manager_running_checker");

    // Query node names (ROS2 replacement for ros::master::getNodes())
    auto node_list = node->get_node_graph_interface()->get_node_names();

    for (const auto& n : node_list)
    {
        if (n == manager_name)
        {
            return true;
        }
    }

    return false;
}
