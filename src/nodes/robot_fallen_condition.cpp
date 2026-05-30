/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Victor Gil
*/

#include "movement_pkg/nodes/robot_fallen_condition.h"

BT::RobotFallenCondition::RobotFallenCondition(
        const std::string &name,
        const BT::NodeConfig& config)
: BT::ConditionNode(name, config)
{
    data_manager_ = config.blackboard->get<std::shared_ptr<CBDataManager>>("data_manager");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("RobotFallen: missing [node] in blackboard");}
}

BT::NodeStatus BT::RobotFallenCondition::tick()
{
    while (rclcpp::ok())
    {
        pitch = data_manager_->getRobotPitch();

        if (present_pitch_ == 0)
            present_pitch_ = pitch;
        else
            present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

        if (present_pitch_ > FALL_FORWARD_LIMIT ||
            present_pitch_ < FALL_BACKWARDS_LIMIT)
        {
            RCLCPP_INFO(node_->get_logger(),
                "Fall detected with pitch: %.2f", present_pitch_);

            RCLCPP_INFO(node_->get_logger(),
                "Robot fallen detected successfully");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

    RCLCPP_ERROR(node_->get_logger(),
        "ROS stopped unexpectedly");

    return BT::NodeStatus::FAILURE;
}
BT::PortsList BT::RobotFallenCondition::providedPorts()
{
    return {};
}