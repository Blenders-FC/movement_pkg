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
            ROS_INFO_STREAM_COND(DEBUG_PRINT, GREEN_TEXT << "[SUCCESS] OP3 manager has finished init pose succesfully!" << RESET_TEXT);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_INFO_COND(!already_logged_ && DEBUG_PRINT, "Waiting for op3 manager to finish init pose");
            already_logged_ = true;
        }
    }

    return BT::FAILURE;  // ROS stopped unexpectedly
}
