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
            goAction(84);  // Left kick
            ros::Duration(0.5).sleep();
            
            DEBUG_STDOUT(get_name() << "Left kick action SUCCESS");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::LeftKick::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("LeftKick HALTED: Stopped walking.");
}