/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/manager_done_condition.h"


BT::ManagerDoneCondition::ManagerDoneCondition(const std::string &name) 
: BT::ConditionNode(name), utils() {}

BT::ReturnStatus BT::ManagerDoneCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        ros::Duration(1).sleep();

        if (checkManagerRunning(manager_name) == true) 
        {
            ROS_INFO_COND(DEBUG_PRINT_, "Succeed to connect");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_WARN("Waiting for op3 manager");
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }

    return BT::FAILURE;
}
