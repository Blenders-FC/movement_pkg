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
    while (ros::ok())
    {

        ball_center_position_ = getBallPosition();
        
        if ((ball_center_position_.x != 999 && ball_center_position_.x != 0) || (ball_center_position_.y != 999 && ball_center_position_.y != 0))
        {   
            ROS_SUCCESS_LOG("BALL detected!");
            ROS_COLORED_LOG("Ball detected with positions: x=%f y=%f", CYAN, false, ball_center_position_.x, ball_center_position_.y);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {   
            ROS_TAGGED_ONCE_LOG("BALL NOT detected", "RED", false, "ball_not_detected_condition");
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
