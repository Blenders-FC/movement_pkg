/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/manager_done_condition.h"


BT::ManagerDoneCondition::ManagerDoneCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::ManagerDoneCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        robot_status_ = getRobotStatus();  // first: module_name  second: status_msg

        if (robot_status_.first == "Base" && robot_status_.second == "Finish Init Pose") 
        {
            ROS_SUCCESS_LOG("OP3 manager has finished init pose succesfully!");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_TAGGED_ONCE_LOG("Waiting for op3 manager to finish init pose", "DEFAULT", false, "Wait_op3_manager");
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
