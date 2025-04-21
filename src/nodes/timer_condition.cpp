/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/timer_condition.h"


BT::TimerCondition::TimerCondition(const std::string &name) 
: BT::ConditionNode(name), duration_sec_(duration) {}


BT::ReturnStatus BT::TimerCondition::Tick()
{
    // Condition checking and state update

    while (ros::ok())
    {
        set_status(BT::RUNNING);

        auto now = std::chrono::steady_clock::now();

        if (!start_time_)
        {
            start_time_ = now;
        }
    
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - *start_time_).count();
    
        if (elapsed >= duration_sec_)
        {
            ROS_SUCCESS_LOG("Timer has finished successfully!");
            set_status(BT::SUCCESS);
            return NodeStatus::SUCCESS;
        }
        else 
        {
            ROS_COLORED_LOG("Waiting for timer: %f segs", CYAN, false, elapsed);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;  
}
