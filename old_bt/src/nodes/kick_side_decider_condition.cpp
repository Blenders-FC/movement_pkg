/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/kick_side_decider_condition.h"


BT::KickSideDeciderCondition::KickSideDeciderCondition(const std::string &name) 
: BT::ConditionNode(name), utils(), gen(rd()), dis(0, 1) {}

BT::ReturnStatus BT::KickSideDeciderCondition::Tick()
{
    // Condition checking and state update
    while(ros::ok())
    {
        // Generate and print the random integer (either 0 or 1)
        _side = dis(gen);
        
        if (_side)
        {
            ROS_COLORED_LOG("RIGHT KICK CHOSEN", CYAN, false); 
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_COLORED_LOG("LEFT KICK CHOSEN", CYAN, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
