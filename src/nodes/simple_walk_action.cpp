/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/simple_walk_action.h"


BT::SimpleWalk::SimpleWalk(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&SimpleWalk::WaitForTick, this);
}

BT::SimpleWalk::~SimpleWalk() {}

void BT::SimpleWalk::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        set_status(BT::RUNNING);
        
        // Perform action...
        while (get_status() != BT::HALTED)
        {
            ROS_TAGGED_ONCE_LOG("Walking...");
            walking_command_ = "start";
            goWalk(walking_command_);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::SimpleWalk::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SimpleWalk HALTED: Stopped walking.");
}
