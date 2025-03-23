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
    while(ros::ok())
    {
        pitch = getRobotPitch();
        
        if (present_pitch_ == 0) 
            present_pitch_ = pitch;
        else
            present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

        if (present_pitch_ > FALL_FORWARD_LIMIT)
        {
            ROS_COLORED_LOG("Forward fall detected with pitch: %f", present_pitch_, CYAN, false);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else if (present_pitch_ < FALL_BACKWARDS_LIMIT) 
        {
            ROS_COLORED_LOG("Backwards fall detected with pitch: %f", present_pitch_, CYAN, false);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_COLORED_LOG("Fall NOT detected.", YELLOW, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}
