/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/can_move_condition.h"


BT::CanMoveCondition::CanMoveCondition(const std::string &name) 
: BT::ConditionNode(name) {
   robot_state_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/via_libre_state", 1, &WalkToTarget::robotStateCallback, this);
}

BT::ReturnStatus BT::CanMoveCondition::Tick()
{
    // Condition checking and state update
    while(ros::ok())
    {        

        if (robot_state_sub_ != -1)
        {
            ROS_SUCCESS_LOG("Robot get valid data to move");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
            
        }
        else
        {
            ROS_COLORED_LOG("Not receiving valid data", CYAN, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
            
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
