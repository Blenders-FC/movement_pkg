/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#ifndef LEFT_KICK_ACTION_H
#define LEFT_KICK_ACTION_H

//#include "movement_pkg/utils.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>
#include "movement_pkg/utils.h"



namespace BT
{
class LeftKick : public BT::StatefulActionNode
{
public:
    // Constructor
    explicit LeftKick(const std::string& name, const NodeConfig& config);
  /*: StatefulActionNode(name, config)
  {
    //node_ = rclcpp::Node::make_shared("left_kick_action");
    //logger_ = node_->get_logger();
  }*/

    static BT::PortsList providedPorts();

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<utils> utils_;
    std::chrono::system_clock::time_point start_time_;
    bool action_sent_ = false;
    bool error_d = false;
};
}  // namespace BT

#endif  // LEFT_KICK_ACTION_H