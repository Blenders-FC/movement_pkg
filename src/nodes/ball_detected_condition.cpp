/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ball_detected_condition.h"


BT::BallDetectedCondition::BallDetectedCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallDetectedCondition::Tick()
{
    // Condition checking and state update

    ball_center_position_ = getBallPosition();
    
    if ((ball_center_position_.x != 999 && ball_center_position_.x != 0) || (ball_center_position_.y != 999 && ball_center_position_.y != 0))
    {
        ROS_INFO("Ball detected with positions: x=%f y=%f", ball_center_position_.x, ball_center_position_.y);
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        ROS_INFO("Ball NOT detected.");
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}
