/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/choose_kick_foot_condition.h"


BT::ChooseKickFootCondition::ChooseKickFootCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::ChooseKickFootCondition::Tick()
{
    // Condition checking and state update
    While(ros::ok())
    {
        head_pan_ = getHeadPan();
        
        if (head_pan_ > 0)
        {
            ROS_COLORED_LOG("RIGHT KICK CHOSEN", CYAN, false); 
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_COLORED_LOG("RIGHT LEFT CHOSEN", CYAN, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}
