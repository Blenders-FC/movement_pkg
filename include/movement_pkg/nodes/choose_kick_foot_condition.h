/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Victor Gil
        */

#ifndef CHOOSE_FOOT_KICK_CONDITION_H
#define CHOOSE_FOOT_KICK_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/condition_node.h"

namespace BT
{
class ChooseKickFootCondition : public BT::ConditionNode, public CBDataManager
{
    public:
        explicit ChooseKickFootCondition(const std::string &name, const BT::NodeConfig& config);  // Constructor

        ~ChooseKickFootCondition() override = default;

        static BT::PortsList providedPorts();

        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        double head_pan_;
        rclcpp::Node::SharedPtr node_;

};
}  // namesapce BT

#endif // CHOOSE_FOOT_KICK_CONDITION_H
