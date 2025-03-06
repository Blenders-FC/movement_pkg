/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/robot_fallen_condition.h"


BT::RobotFallenCondition::RobotFallenCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::RobotFallenCondition::Tick()
{
    // Condition checking and state update
    
    pitch = getRobotPitch();
    
    if (present_pitch_ == 0) 
        present_pitch_ = pitch;
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    if (present_pitch_ > FALL_FORWARD_LIMIT)
    {
        ROS_INFO("Forward fall detected with pitch: %f", present_pitch_);
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else if (present_pitch_ < FALL_BACKWARDS_LIMIT) 
    {
        ROS_INFO("Backwards fall detected with pitch: %f", present_pitch_);
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        ROS_INFO("Fall NOT detected.");
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}
