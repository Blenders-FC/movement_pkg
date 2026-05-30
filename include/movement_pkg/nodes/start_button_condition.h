/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef START_BUTTON_CONDITION_H
#define START_BUTTON_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/condition_node.h"


namespace BT
{
class StartButtonCondition : public ConditionNode
{
    public:
        explicit StartButtonCondition(const std::string &name, const BT::NodeConfig& config);  // Constructor

        ~StartButtonCondition() override = default;
        // Behavior Tree Tick function
        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        //button
        bool start_button_flag_ = false;
        bool already_logged_ = false;
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<CBDataManager> data_manager_;
};
}  // namesapce BT

#endif  // START_BUTTON_CONDITION_H
