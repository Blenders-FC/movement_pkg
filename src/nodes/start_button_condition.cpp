/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/start_button_condition.h"


BT::StartButtonCondition::StartButtonCondition(const std::string &name, const BT::NodeConfig& config) 
: BT::ConditionNode(name, config)
{
    data_manager_ = config.blackboard->get<std::shared_ptr<CBDataManager>>("data_manager");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("StartButton: missing [node] in blackboard");}
}

BT::NodeStatus BT::StartButtonCondition::tick()
{
    // Condition checking and state update

    if (rclcpp::ok())
    {
        //set_status(BT::RUNNING);
        start_button_flag_ = data_manager_->getStartButtonState();

        if (start_button_flag_)
        {
            asm("NOP");
            //ROS_SUCCESS_LOG("Start Button ready! Start moving...");
            //set_status(BT::SUCCESS);
            RCLCPP_INFO(node_->get_logger(), "Start Button ready! Start moving...");
            return BT::NodeStatus::SUCCESS;
        } 
        else 
        {
            RCLCPP_WARN(node_->get_logger(), "Waiting for start button");
        }
    }
    RCLCPP_ERROR(node_->get_logger(), "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE;  
}
BT::PortsList BT::StartButtonCondition::providedPorts()
{
    return {};
}
