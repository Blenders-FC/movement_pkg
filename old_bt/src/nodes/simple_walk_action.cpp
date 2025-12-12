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
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_simple_walk");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_simple_walk");
        
        // Perform action...
        while (get_status() == BT::IDLE)
        {
            ROS_TAGGED_ONCE_LOG("Walking...", "TEAL", true, "Start_simple_walk");
            walking_command_ = "start";
            goWalk(walking_command_);
            set_status(BT::RUNNING);

            ROS_SUCCESS_LOG("Simple walk has started successfully!");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::SimpleWalk::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SimpleWalk HALTED: Stopped simple walking", "ORANGE", false, "Halted_simple_walking");
}
