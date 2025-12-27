/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
*/

#include "movement_pkg/nodes/right_kick_action.h"
using namespace std::chrono_literals;

namespace BT{
RightKick::RightKick(
    const std::string& name,
    const BT::NodeConfig& config) 
: StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("right_kick_action");
    RCLCPP_INFO(node_->get_logger(), "RightKick constructed");
}

NodeStatus RightKick::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[RightKick] START");

    // reset action flag
    action_sent_ = false;
    start_time_ = std::chrono::system_clock::now();

    return NodeStatus::RUNNING;
}

BT::NodeStatus RightKick::onRunning()
{
    if (!action_sent_)
    {
        RCLCPP_INFO(node_->get_logger(), "[RightKick] Executing RIGHT KICK...");
        // Your action call here
        // goAction(83);  
        action_sent_ = true;
        
        return BT::NodeStatus::RUNNING;
    }
    auto elapsed = std::chrono::system_clock::now() - start_time_;

    if (elapsed > 1s){
        RCLCPP_INFO(node_->get_logger(), "[RightKick] DONE");
        return NodeStatus::SUCCESS;
    }

    if (error_d){
        return BT::NodeStatus::IDLE;
    }
    return BT::NodeStatus::RUNNING;

    
}
BT::PortsList BT::RightKick::providedPorts()
{
    return {};
}

void RightKick::onHalted()
{
    
    RCLCPP_INFO(node_->get_logger(), "[RightKick] HALTED: Stopped right kicking");
    error_d = true;
}
} //namespace BT
