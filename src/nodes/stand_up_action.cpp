/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/stand_up_action.h"


BT::WalkToTarget::WalkToTarget(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToTarget::WaitForTick, this);
}

BT::WalkToTarget::~WalkToTarget() {}

void BT::WalkToTarget::WaitForTick()
{
    DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
    tick_engine.Wait();
    DEBUG_STDOUT(get_name() << "TICK RECEIVED");

    set_status(BT::RUNNING);

    stopWalking();
    ros::Duration(1.0).sleep();
    
    set_status(BT::SUCCESS);
}

void BT::WalkToTarget::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    DEBUG_STDOUT("WalkToTarget HALTED: Stopped walking.");
}
