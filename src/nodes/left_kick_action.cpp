/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movemente_pkg/nodes/left_kick_action.h>

BT::LeftKickAction::LeftKickAction(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&LeftKickAction::WaitForTick, this);
}

BT::LeftKickAction::~LeftKickAction() {}

void BT::LeftKickAction::WaitForTick()
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
            return BT::SUCCESS;
        }
    }
}

void BT::LeftKickAction::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("LeftKickAction HALTED: Stopped walking.");
}