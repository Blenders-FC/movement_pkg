/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ref_entry_condition.h"


BT::RefEntryCondition::RefEntryCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::RefEntryCondition::Tick()
{

    while (ros::ok())
    {
            // Condition checking and state update
        int refereeState= blackboard.getTarget("m_refereeStatus")->refereeStatus;   

        // if (refereeState == referee::STILL || refereeState == referee::GET_FAR)
        // {   
        //     ROS_COLORED_LOG("Not allowed to play by referee",CYAN, true);
        //     set_status(BT::FAILURE);
        //     return BT::FAILURE;

        // }
        // else
        // {   
        //     ROS_SUCCESS_LOG("Play");
        //     ROS_COLORED_LOG("referee state  is %d",CYAN, true, refereeState);
        //     set_status(BT::SUCCESS);
        //     return BT::SUCCESS;
        // }
        if (refereeState == referee::MIDFIELD || refereeState == referee::PLAY ) //midfield is ready 
        {   
            ROS_SUCCESS_LOG("Ready to play");
            ROS_COLORED_LOG("referee state  is %d",CYAN, true, refereeState);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;

        }
        else
        {   
            ROS_TAGGED_ONCE_LOG("Waiting for READY from referee", "DEFAULT", true);
            // set_status(BT::FAILURE);
            // return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
