/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/stand_up_action.h"


BT::StandUp::StandUp(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&StandUp::WaitForTick, this);
}

BT::StandUp::~StandUp() {}

void BT::StandUp::WaitForTick()
{
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        set_status(BT::RUNNING);

        if (get_status() != BT::HALTED)
        {
            stopWalking();
            ros::Duration(1.0).sleep();
            
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
    }
}

void BT::StandUp::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("StandUp HALTED: Stopped walking.");
}
