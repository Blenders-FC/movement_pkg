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
            ROS_INFO_STREAM_COND(DEBUG_PRINT, GREEN_TEXT << "[SUCCESS] Start Button ready! Start moving..." << RESET_TEXT);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        } 
        else 
        {
            ROS_INFO_COND(!already_logged_ && DEBUG_PRINT, "Waiting for start button");
            already_logged_ = true;
        }
    }

    return BT::FAILURE;  // ROS stopped unexpectedly 
}
