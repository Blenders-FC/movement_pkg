/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movemente_pkg/nodes/right_kick_action.h>

BT::RightKickAction::RightKickAction(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&RightKickAction::WaitForTick, this);
}

BT::RightKickAction::~RightKickAction() {}

void BT::RightKickAction::WaitForTick()
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
            goAction(83);  // right kick
            ros::Duration(0.5).sleep();
            
            DEBUG_STDOUT(get_name() << "Right kick action SUCCESS");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
    }
}

void BT::RightKickAction::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("RightKickAction HALTED: Stopped walking.");
}