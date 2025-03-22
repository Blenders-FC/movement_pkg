/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/get_up_forward_action.h"

BT::GetUpForward::GetUpForward(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&GetUpForward::WaitForTick, this);
}

BT::GetUpForward::~GetUpForward() {}

void BT::GetUpForward::WaitForTick()
{
    while(ros::ok())
    {
        while (true)
        {
            // Waiting for the first tick to come
            ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
            tick_engine.Wait();
            ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

            // Running state
            set_status(BT::RUNNING);

            // Perform action...
            if (get_status() != BT::HALTED)
            {
                goAction(122);  // get up forward
                ros::Duration(0.5).sleep();
                
                ROS_SUCCESS_LOG("Get up forward action SUCCESS");
                set_status(BT::SUCCESS);
            }
        }
    }

    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}

void BT::GetUpForward::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("GetUpForward HALTED: Stopped walking.");
}