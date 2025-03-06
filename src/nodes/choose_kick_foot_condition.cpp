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

    head_pan_ = getHeadPan();
    
    if (head_pan_ > 0)
    {
        ROS_INFO("RIGHT KICK CHOSEN");
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        ROS_INFO("RIGHT LEFT CHOSEN");
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}
