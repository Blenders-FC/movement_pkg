/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_middle_field_condition.h"


BT::WalkMiddleFieldCondition::WalkMiddleFieldCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::WalkMiddleFieldCondition::Tick()
{

    while (ros::ok())
    {
            // Condition checking and state update
        int midddleFieldState = blackboard.getTarget("middle_field_placement")->middle_field_placement;   

        if (midddleFieldState == true)
        {   
            ROS_SUCCESS_LOG("Go to middle field");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {   
            ROS_COLORED_LOG("No go to middle field",CYAN, true);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
