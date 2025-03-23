/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/left_kick_action.h"

BT::LeftKick::LeftKick(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&LeftKick::WaitForTick, this);
}

BT::LeftKick::~LeftKick() {}

void BT::LeftKick::WaitForTick()
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
        if (get_status() != BT::HALTED)
        {
            goAction(84);  // Left kick
            ros::Duration(0.5).sleep();
            
            ROS_SUCCESS_LOG("Left kick actions");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}

void BT::LeftKick::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("LeftKick HALTED: Stopped walking.");
}