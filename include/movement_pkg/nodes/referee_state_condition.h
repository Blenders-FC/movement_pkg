/*
    Authors:
        Pedro Deniz
        Marlene Cobian

    Iván Delgado
*/

#ifndef REFEREE_STATE_CONDITION_H
#define REFEREE_STATE_CONDITION_H

#include "movement_pkg/utils.h"
#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/condition_node.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


namespace BT
{
class RefereeStateCondition : public BT::ConditionNode
{
    public:
        explicit RefereeStateCondition(const std::string &name, const BT::NodeConfig& config);  // Constructor

        ~RefereeStateCondition() override = default;

        static BT::PortsList providedPorts();


        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        
        CBDataManager cb_data_manager_;
};
}  // namesapce BT

#endif // RefereeStateCondition