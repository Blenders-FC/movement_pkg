
/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Victor Gil
*/

#ifndef BALL_DETECTED_CONDITION_H
#define BALL_DETECTED_CONDITION_H

#include "movement_pkg/utils.h"
#include "movement_pkg/cb_data_manager.h"
#include <behaviortree_cpp/bt_factory.h>
#include "behaviortree_cpp/condition_node.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>


namespace BT
{
class BallDetectedCondition : public BT::ConditionNode
{
    public:
        explicit BallDetectedCondition(const std::string& name, const BT::NodeConfig& config);  // Constructor


        ~BallDetectedCondition() override = default;

        static BT::PortsList providedPorts();

        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<utils> utils_;
        geometry_msgs::msg::Point ball_center_position_;
	    std::shared_ptr<CBDataManager> data_manager_;
};
}  // namesapce BT

#endif  // BALL_DETECTED_CONDITION_H
