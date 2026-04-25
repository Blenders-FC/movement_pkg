/*
    Authors:
        Pedro Deniz
        Marlene Cobian
	
	Victor Gil
*/

#include "movement_pkg/nodes/ball_detected_condition.h"


BT::BallDetectedCondition::BallDetectedCondition(
		const std::string &name, 
		const BT::NodeConfig& config
		) 
: BT::ConditionNode(name, config)
{
	if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("BallDetected: missing [node] in blackboard");
}
    utils_ = std::make_shared<utils>(node_);
}

BT::NodeStatus BT::BallDetectedCondition::tick()
{
    // Condition checking and state update
    while (rclcpp::ok())
    {

        ball_center_position_ = cb_data_manager_.getBallPosition();
        if ((ball_center_position_.x != 999 && ball_center_position_.x != 0) || (ball_center_position_.y != 999 && ball_center_position_.y != 0))
        {   
            RCLCPP_INFO(rclcpp::get_logger("BallDetectedCondition"), 
			    "Ball Detected!");
            RCLCPP_INFO(rclcpp::get_logger("BallDetectedCondition"),
			    "Ball detected with positions: x=%f y=%f", ball_center_position_.x, ball_center_position_.y);
            //set_status(BT::SUCCESS);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {   
            RCLCPP_INFO(rclcpp::get_logger("BallDetectedCondition"), 
			    "Ball NOT Detected!");
            //set_status(BT::FAILURE);
            return BT::NodeStatus::FAILURE;
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("BallDetectedCondition"), 
		    "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE; 
}

BT::PortsList BT::BallDetectedCondition::providedPorts()
{
    return {};
}
