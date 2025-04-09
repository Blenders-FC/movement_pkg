/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/get_up_backwards_action.h"

BT::GetUpBackwards::GetUpBackwards(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&GetUpBackwards::WaitForTick, this);
}

BT::GetUpBackwards::~GetUpBackwards() {}

void BT::GetUpBackwards::WaitForTick()
{
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        // Perform action...
        while (get_status() != BT::HALTED)
        {
            goAction(1);  // straighten legs
            ros::Duration(0.5).sleep();
            goAction(82);  // get up backwards
            ros::Duration(0.5).sleep();
            
            ROS_SUCCESS_LOG("Get up backwards action");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::GetUpBackwards::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("GetUpBackwards HALTED: Stopped walking.");
}