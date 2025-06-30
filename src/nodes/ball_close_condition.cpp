/*
    Authors:
        Victor Gil
*/

#include "movement_pkg/nodes/ball_close_condition.h"


BT::BallCloseCondition::BallCloseCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallCloseCondition::Tick()
{
    // Condition checking and state update
    while(ros::ok())
    {
        ballArea = getBallArea();
        
        if (ballArea < 500) // ??
        {
            ROS_COLORED_LOG("Ball NOT CLOSE enough!", YELLOW, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
        else
        {
            ROS_SUCCESS_LOG("Ball is CLOSE");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
