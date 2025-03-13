/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/manager_running_condition.h"


BT::ManagerRunningCondition::ManagerRunningCondition(const std::string &name) 
: BT::ConditionNode(name), utils() {}

BT::ReturnStatus BT::ManagerRunningCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        ros::Duration(1).sleep();

        if (checkManagerRunning(manager_name) == true) 
        {
            ROS_INFO_STREAM_COND(DEBUG_PRINT, GREEN_TEXT << "[SUCCESS] Succeed to connect to OP3 manager!" << RESET_TEXT);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_INFO_COND(!already_logged_ && DEBUG_PRINT, "Waiting for op3 manager to connect");
            already_logged_ = true;
        }
    }

    return BT::FAILURE;  // ROS stopped unexpectedly
}

bool BT::ManagerRunningCondition::checkManagerRunning(std::string& manager_name) {
    std::vector<std::string> node_list;
    ros::master::getNodes(node_list);
  
    for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
    {
      if (node_list[node_list_idx] == manager_name)
        return true;
    }
    ROS_ERROR("Can't find op3_manager");
    return false;
}
