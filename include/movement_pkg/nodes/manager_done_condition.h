/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef MANAGER_DONE_CONDITION_H
#define MANAGER_DONE_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/condition_node.h"


namespace BT
{
class ManagerDoneCondition : public ConditionNode
{
    public:
        explicit ManagerDoneCondition(const std::string &name, const BT::NodeConfig& config);  // Constructor

        static BT::PortsList providedPorts();
        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        bool already_logged_ = false;
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<CBDataManager> data_manager_;
        std::pair<std::string, std::string> robot_status_;
};
}  // namesapce BT

#endif  // MANAGER_DONE_CONDITION_H
