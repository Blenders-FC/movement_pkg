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
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_l_kick");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_l_kick");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // Running state
            set_status(BT::RUNNING);

            goAction(84);  // Left kick
            ros::Duration(0.5).sleep();
            
            ROS_SUCCESS_LOG("Left kick actions");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::LeftKick::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("LeftKick HALTED: Stopped left kicking", "ORANGE", false, "Halted_l_kick");
}