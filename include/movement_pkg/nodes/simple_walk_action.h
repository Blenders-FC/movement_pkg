/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#ifndef SIMPLE_WALK_ACTION_H
#define SIMPLE_WALK_ACTION_H

#include <chrono>
#include <memory>
#include <thread>
#include "movement_pkg/walking_controller.h"
#include "behaviortree_cpp/action_node.h"



namespace BT
{
class SimpleWalk : public BT::StatefulActionNode
{
public:
    // Constructor
    explicit SimpleWalk(const std::string& name, const NodeConfig& config);
    ~SimpleWalk() override;
    static BT::PortsList providedPorts();

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<utils> utils_;
    std::shared_ptr<WalkingController> walking_controller_;
    std::string walking_command_;
    bool action_sent_ = false;
    bool error_d = false;
};
}  // namespace BT

#endif  // SIMPLE_WALK_ACTION_H