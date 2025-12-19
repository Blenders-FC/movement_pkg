/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#include "movement_pkg/nodes/left_kick_action.h"
using namespace std::chrono_literals;

namespace BT{
LeftKick::LeftKick(
    const std::string& name,
    const BT::NodeConfig& config) 
: StatefulActionNode(name, config)
{
    
    node_ = rclcpp::Node::make_shared("left_kick_action");
    RCLCPP_INFO(node_->get_logger(), "LeftKick constructed");
}

NodeStatus LeftKick::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[LeftKick] START");

    // reset action flag
    action_sent_ = false;
    start_time_ = std::chrono::system_clock::now();

    return NodeStatus::RUNNING;
}

BT::NodeStatus LeftKick::onRunning()
{
    if (!action_sent_)
    {
        RCLCPP_INFO(node_->get_logger(), "[LeftKick] Executing LEFT KICK...");
        // Your action call here
        // goAction(84);  
        action_sent_ = true;
        
        return BT::NodeStatus::RUNNING;
    }
    auto elapsed = std::chrono::system_clock::now() - start_time_;

    if (elapsed > 1s){
        RCLCPP_INFO(node_->get_logger(), "[LeftKick] DONE");
        return NodeStatus::SUCCESS;
    }

    if (error_d){
        return BT::NodeStatus::IDLE;
    }
    return BT::NodeStatus::RUNNING;

    
}
BT::PortsList BT::LeftKick::providedPorts()
{
    return {};
}

void LeftKick::onHalted()
{
    
    RCLCPP_INFO(node_->get_logger(), "[LeftKick] HALTED: Stopped left kicking");
    error_d = true;
}
} //namespace BT
/*BT_RegisterNodesFromPlugin(factory)
{
    factory.registerNodeType<BT::LeftKick>("LeftKick");
}*/