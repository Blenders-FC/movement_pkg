/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ball_in_center_condition.h"


BT::BallInCenterCondition::BallInCenterCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallInCenterCondition::Tick()
{
    // Condition checking and state update

    ball_center_position_ = getBallPosition();
    xerror_ = (320 - ball_center_position_.x) * 0.21875;  // 70 / 320
    yerror_ = (240 - ball_center_position_.y) * 0.29166;  // 70 / 240
    
    if (xerror_ <= 20 || xerror_ >= -20) && (yerror_ <= 20 || yerror_ >= -20)
    {
        ROS_INFO("Ball NOT in center! Starting centering process!");
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
    else
    {
        ROS_INFO("Ball IN CENTER! Starting walking process!");
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
}
