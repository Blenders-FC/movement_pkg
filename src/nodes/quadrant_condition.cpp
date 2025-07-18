/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/quadrant_condition.h"


BT::QuadrantCondition::QuadrantCondition(const std::string &name) 
: BT::ConditionNode(name), utils() {}

BT::ReturnStatus BT::QuadrantCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        if (quadrant % 2 == 1)  // odd quadrants (1,3)
        {   
            ROS_COLORED_LOG("Odd quadrant, turning right!", VIOLET, false);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {   
            ROS_COLORED_LOG("Even quadrant, turning left!", VIOLET, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}