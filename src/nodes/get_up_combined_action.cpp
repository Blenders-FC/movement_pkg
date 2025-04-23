/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/get_up_combined_action.h"

BT::GetUpCombined::GetUpCombined(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&GetUpCombined::WaitForTick, this);
}

BT::GetUpCombined::~GetUpCombined() {}

void BT::GetUpCombined::WaitForTick()
{
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // Running state
            set_status(BT::RUNNING);

            goAction(1);  // straighten legs
            ros::Duration(0.5).sleep();
            
            pitch = getRobotPitch();
        
            if (present_pitch_ == 0) 
                present_pitch_ = pitch;
            else
                present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;


            if (present_pitch_ > FALL_FORWARD_LIMIT)
            {
                ROS_COLORED_LOG("Forward fall detected with pitch: %f", CYAN, true, present_pitch_);
                goAction(122);  // get up forward
                ros::Duration(1).sleep();

                ROS_SUCCESS_LOG("Get up forwards action");
                set_status(BT::SUCCESS);
            }
            else if (present_pitch_ < FALL_BACKWARDS_LIMIT) 
            {
                ROS_COLORED_LOG("Backwards fall detected with pitch: %f", CYAN, true, present_pitch_);
                goAction(82);  // get up forward
                ros::Duration(1).sleep();

                ROS_SUCCESS_LOG("Get up forwards action");
                set_status(BT::SUCCESS);
            }
            else
            {
                ROS_COLORED_LOG("Fall NOT detected", YELLOW, false);
                set_status(BT::FAILURE);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::GetUpCombined::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("GetUpCombined HALTED: Stopped walking.");
}