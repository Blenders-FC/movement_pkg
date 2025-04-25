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
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_getup_forward");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_getup_forward");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // Running state
            set_status(BT::RUNNING);

            goAction(1);  // straighten legs
            ros::Duration(0.5).sleep();
            goAction(122);  // get up forward
            ros::Duration(0.5).sleep();
            
            ROS_SUCCESS_LOG("Get up forwards action");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::GetUpForward::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("GetUpForward HALTED: Stopped get up forward", "ORANGE", false, "Halted_getup_forward");
}