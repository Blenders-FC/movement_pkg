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
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        set_status(BT::RUNNING);
        
        // Perform action...
        while (get_status() != BT::HALTED)
        {
            DEBUG_STDOUT(get_name() << "Walking...");
            walk_command = "start";
            goWalk(walk_command);
        }
    }
}

void BT::SimpleWalk::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    DEBUG_STDOUT("SimpleWalk HALTED: Stopped walking.");
}
