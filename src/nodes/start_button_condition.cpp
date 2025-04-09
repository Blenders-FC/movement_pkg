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
            ROS_SUCCESS_LOG("Start Button ready! Start moving...");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        } 
        else 
        {
            ROS_TAGGED_ONCE_LOG("Waiting for start button");
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;  
}
