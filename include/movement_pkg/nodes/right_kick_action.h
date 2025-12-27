/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#ifndef RIGHT_KICK_ACTION_H
#define RIGHT_KICK_ACTION_H

//#include "movement_pkg/utils.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <thread>



namespace BT
{
class RightKick : public BT::StatefulActionNode
{
public:
    // Constructor
    explicit RightKick(const std::string& name, const NodeConfig& config);
  /*: StatefulActionNode(name, config)
  {
    //node_ = rclcpp::Node::make_shared("right_kick_action");
    //logger_ = node_->get_logger();
  }*/

    static BT::PortsList providedPorts();

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    std::chrono::system_clock::time_point start_time_;
    bool action_sent_ = false;
    bool error_d = false;
};
}  // namespace BT

#endif  // RIGHT_KICK_ACTION_H
