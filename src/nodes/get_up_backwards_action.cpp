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
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        // Perform action...
        if (get_status() != BT::HALTED)
        {
            goAction(1);  // straighten legs
            ros::Duration(0.5).sleep();
            goAction(82);  // get up backwards
            ros::Duration(0.5).sleep();
            
            DEBUG_STDOUT(get_name() << "Get up backwards action SUCCESS");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::GetUpBackwards::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("GetUpBackwards HALTED: Stopped walking.");
}