/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/start_button_condition.h"


BT::StartButtonCondition::StartButtonCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::StartButtonCondition::Tick()
{
    // Condition checking and state update

    while (ros::ok())
    {
        set_status(BT::RUNNING);
        ros::spinOnce();
        start_button_flag_ = getStartButtonState();

        if (start_button_flag_)
        {
            asm("NOP");
            ROS_INFO_COND(DEBUG_PRINT_, "MOVE!");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        } 
        else 
        {
            ROS_INFO_COND(DEBUG_PRINT_, "WAITING FOR BUTTON");
        }
    }

    return BT::FAILURE;  // ROS stopped unexpectedly 
}
