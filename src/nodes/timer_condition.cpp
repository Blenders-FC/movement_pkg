/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/timer_condition.h"


BT::TimerCondition::TimerCondition(const std::string &name, double duration) 
: BT::ConditionNode(name), duration_sec_(duration), timer_started_(false) {}


BT::ReturnStatus BT::TimerCondition::Tick()
{
    // Condition checking and state update

    while (ros::ok())
    {
        if (!timer_started_)
        {
            _start_time = std::chrono::steady_clock::now();
            timer_started_ = true;
            ROS_COLORED_LOG("Timer started: %f secs", PINK, false, duration_sec_);
        }

        auto now = std::chrono::steady_clock::now();

        // Reset timer if node was just restarted
        if (get_status() == BT::IDLE || !timer_started_) {
            _start_time = now;
            timer_started_ = true;
            set_status(BT::RUNNING);  // Ensure it's running while counting
            ROS_COLORED_LOG("Timer started: %f", PINK, false, duration_sec_);
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - _start_time).count();

        if (elapsed >= duration_sec_)
        {
            ROS_SUCCESS_LOG("Timer has finished successfully!");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else 
        {
            ROS_COLORED_LOG("Waiting for timer: %ld segs", CYAN, false, elapsed);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;  
}
