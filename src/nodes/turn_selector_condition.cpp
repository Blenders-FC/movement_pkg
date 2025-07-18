/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/turn_selector_condition.h"


BT::TurnSelectorCondition::TurnSelectorCondition(const std::string &name) 
    : BT::ConditionNode(name) {}

BT::ReturnStatus BT::TurnSelectorCondition::Tick()
{

    // Condition checking and state update

    while (ros::ok())
    {
        pan_angle_to_ball = getHeadPan();

        if(pan_angle_to_ball < 0)  // right side
        {
            set_status(BT::SUCCESS);
            ROS_SUCCESS_LOG("Ball is on the right!");
            return BT::SUCCESS;
        }
        else   // left side
        {
            set_status(BT::FAILURE);
            ROS_SUCCESS_LOG("Ball is on the LEFT!");
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
