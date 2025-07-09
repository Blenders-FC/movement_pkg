/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/init_pose_condition.h"


BT::InitPoseCondition::InitPoseCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::InitPoseCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        is_valid_pose_ = isInitPoseValid();
        
        if (is_valid_pose_)
        {   
            ROS_SUCCESS_LOG("Valid init pose!");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {   
            ROS_COLORED_LOG("Not a valid init pose. Searching goals...", RED, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
