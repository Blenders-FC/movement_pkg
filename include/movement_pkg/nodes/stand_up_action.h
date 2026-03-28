/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Iván Delgado
*/

#ifndef STAND_UP_ACTION_H
#define STAND_UP_ACTION_H

#include "movement_pkg/utils.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include "movement_pkg/walking_controller.h"
#include <chrono>
#include <memory>
#include <thread>


namespace BT
{
class StandUp : public BT::StatefulActionNode
{
    public:
        // Constructor
        explicit StandUp(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

        NodeStatus onStart() override;
        NodeStatus onRunning() override; // The method that is going to be executed by the thread
        void onHalted() override; // The method used to interrupt the execution of the node

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<utils> utils_;
        std::shared_ptr<WalkingController> walking_controller_;
        std::chrono::system_clock::time_point start_time_;
        bool action_sent_ =false;
        bool error_d = false;
};
}  // namespace BT

#endif  // STAND_UP_ACTION_H