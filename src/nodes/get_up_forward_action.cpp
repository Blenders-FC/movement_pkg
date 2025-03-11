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
            goAction(122);  // get up forward
            ros::Duration(0.5).sleep();
            
            DEBUG_STDOUT(get_name() << "Get up forward action SUCCESS");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::GetUpForward::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("GetUpForward HALTED: Stopped walking.");
}