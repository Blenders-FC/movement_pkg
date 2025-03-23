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
    while (ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        set_status(BT::RUNNING);

        if (get_status() != BT::HALTED)
        {
            stopWalking();
            // ros::Duration(1.0).sleep();  // Verify if it's completely necessary

            ROS_SUCCESS_LOG("Succeed to stand up!");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}

void BT::StandUp::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("StandUp HALTED: Stopped walking.");
}
