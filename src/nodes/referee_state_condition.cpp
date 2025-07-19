/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/referee_state_condition.h"


BT::RefereeStateCondition::RefereeStateCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::RefereeStateCondition::Tick()
{

    while (ros::ok())
    {
            // Condition checking and state update
        int refereeState = blackboard.getTarget("m_refereeStatus")->refereeStatus;   

        if (refereeState == referee::STILL || refereeState == referee::GET_FAR)
        {   
            ROS_COLORED_LOG("Not allowed to play by referee",CYAN, true);
            set_status(BT::FAILURE);
            return BT::FAILURE;

        }
        else
        {   
            ROS_SUCCESS_LOG("Play");
            ROS_COLORED_LOG("referee state  is %d",CYAN, true, refereeState);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
