/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Victor Gil  
*/

#include "movement_pkg/nodes/choose_kick_foot_condition.h"

BT::ChooseKickFootCondition::ChooseKickFootCondition(
        const std::string& name,
        const BT::NodeConfiguration& config
        )
: BT::ConditionNode(name, config) {
  data_manager_ = config.blackboard->get<std::shared_ptr<CBDataManager>>("data_manager");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("ChooseKickFoot: missing [node] in blackboard");}

}

BT::NodeStatus BT::ChooseKickFootCondition::tick()
{
  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(node_->get_logger(), "ROS2 stopped unexpectedly");
    return BT::NodeStatus::FAILURE;
  }

  double head_pan = data_manager_->getHeadPan();

  if (head_pan < 0.0)
  {
    RCLCPP_INFO(node_->get_logger(), "RIGHT KICK CHOSEN");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "LEFT KICK CHOSEN");
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList BT::ChooseKickFootCondition::providedPorts()
{
    return {};
}
