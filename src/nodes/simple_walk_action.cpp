/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#include "movement_pkg/nodes/simple_walk_action.h"
using namespace std::chrono_literals;

namespace BT{
SimpleWalk::SimpleWalk(
    const std::string& name,
    const BT::NodeConfig& config) 
: StatefulActionNode(name, config)
{

    //node_ = rclcpp::Node::make_shared("simple_walk_action");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("SimpleWalk: missing [node] in blackboard");
}
    utils_ = std::make_shared<utils>(node_);
    RCLCPP_INFO(node_->get_logger(), "SimpleWalk constructed");
}

NodeStatus SimpleWalk::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[SimpleWalk] START");

    // reset action flag
    action_sent_ = false;
    //start_time_ = std::chrono::system_clock::now();
    RCLCPP_INFO(node_->get_logger(), "[SimpleWalk] Walking...");
    walking_command_ = "start";
    walking_controller_->goWalk(walking_command_);
    RCLCPP_INFO(node_->get_logger(), "[SimpleWalk] Simple walk has started successfully!");
    return NodeStatus::RUNNING;
}

BT::NodeStatus SimpleWalk::onRunning()
{
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[SimpleWalk] Walking..."
    );    
            
            /* setStatus(BT::NodeStatus::RUNNING);

            RCLCPP_INFO(node_->get_logger(), "[SimpleWalk] Simple walk has started successfully!");
            setStatus(BT::NodeStatus::SUCCESS); */
        
    if (error_d){
        return BT::NodeStatus::IDLE;
    }
    return NodeStatus::RUNNING;

}
BT::PortsList BT::SimpleWalk::providedPorts()
{
    return {};
}

void SimpleWalk::onHalted()
{
    walking_controller_->stopWalking();
    RCLCPP_INFO(node_->get_logger(), "[SimpleWalk] HALTED: Stopped simple walking");
    error_d = true;
}
} //namespace BT

namespace BT {
SimpleWalk::~SimpleWalk() = default;
}