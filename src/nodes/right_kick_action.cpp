/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/nodes/right_kick_action.h>

BT::RightKick::RightKick(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&RightKick::WaitForTick, this);
}

BT::RightKick::~RightKick() {}

void BT::RightKick::WaitForTick()
{
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_r_kick");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_r_kick");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // Running state
            set_status(BT::RUNNING);

            goAction(83);  // right kick
            ros::Duration(0.5).sleep();
            
            ROS_SUCCESS_LOG("Right kick action");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::RightKick::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("RightKick HALTED: Stopped right kick", "ORANGE", false, "Halted_r_kick");
}