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
    //node_ = rclcpp::Node::make_shared("manager_done_condition");
    data_manager_ = config.blackboard->get<std::shared_ptr<CBDataManager>>("data_manager");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("ManagerDone: missing [node] in blackboard");}
    // Register this node with the shared executor so callbacks fire
    //init();
    //auto executor = config.blackboard->get<std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>>("executor");
    //executor->add_node(this->shared_from_this());  // <<< this is what was missing
}

BT::NodeStatus BT::ManagerDoneCondition::tick()
{
    // Condition checking and state update
    while (rclcpp::ok())
    {
        robot_status_ = data_manager_->getRobotStatus();  // first: module_name  second: status_msg

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
            RCLCPP_WARN(node_->get_logger(), "Current Pose: %s, Status: %s", robot_status_.first.c_str(), robot_status_.second.c_str());
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE;
}
BT::PortsList BT::ManagerDoneCondition::providedPorts()
{
    return {};
}
