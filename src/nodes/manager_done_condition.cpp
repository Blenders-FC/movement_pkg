/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/manager_done_condition.h"


BT::ManagerDoneCondition::ManagerDoneCondition(
    const std::string &name, const BT::NodeConfig& config)
: BT::ConditionNode(name, config) 
{
    node_ = rclcpp::Node::make_shared("manager_done_condition");
}

BT::NodeStatus BT::ManagerDoneCondition::tick()
{
    // Condition checking and state update
    while (rclcpp::ok())
    {
        robot_status_ = getRobotStatus();  // first: module_name  second: status_msg

        if (robot_status_.first == "Base" && robot_status_.second == "Finish Init Pose") 
        {
            //ROS_SUCCESS_LOG("OP3 manager has finished init pose succesfully!");
            RCLCPP_INFO(node_->get_logger(), "OP3 manager has finished init pose succesfully!");
            //set_status(BT::SUCCESS);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Waiting for op3 manager to finish init pose");
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE;
}
