/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Ricardo Berumen
*/

#ifndef MANAGER_RUNNING_CONDITION_H
#define MANAGER_RUNNING_CONDITION_H

//#include "movement_pkg/utils.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


namespace BT
{
class ManagerRunningCondition : public BT::ConditionNode
{
    public:
        explicit ManagerRunningCondition(const std::string& name, const BT::NodeConfig& config);  // Constructor


        ~ManagerRunningCondition() override = default;

        static BT::PortsList providedPorts();

        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        bool checkManagerRunning(std::string& manager_name);

        std::string manager_name = "/op3_manager";
        bool already_logged_ = false;
        rclcpp::Node::SharedPtr node_;
};
}  // namesapce BT

#endif  // MANAGER_RUNNING_CONDITION_H