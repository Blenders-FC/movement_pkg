/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Ivan Delgado
*/

#include "movement_pkg/nodes/ball_detected_condition2.h"


BT::BallDetectedCondition2::BallDetectedCondition2(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallDetectedCondition2::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {

        ball_center_position_ = getBallPosition();
        
        if ((ball_center_position_.z > (44000)))
        {   
            ROS_SUCCESS_LOG("BALL detected!");
            ROS_COLORED_LOG("Ball detected with area: z=%f", CYAN, false, ball_center_position_.z);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {   
            ROS_COLORED_LOG("BALL NOT detected", RED, false);          
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
