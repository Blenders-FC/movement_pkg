/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_distance_action.h"


BT::WalkToDistance::WalkToDistance(std::string name, double distance, bool reset)
: ActionNode::ActionNode(name), WalkingController(), distance_to_walk(distance), reset_(reset)
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToDistance::WaitForTick, this);
}

BT::WalkToDistance::~WalkToDistance() {}

void BT::WalkToDistance::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_walk_distance");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_walk_distance");

        // Perform action...
        walked_distance = 0;  // Resets in each cycle

        while (get_status() == BT::IDLE)
        {
            set_status(BT::RUNNING);

            this->setModule("walking_module");
            ROS_TAGGED_ONCE_LOG("Walking towards distance...", "BROWN_BG", true, "Walk_distance");
            walkTowardsDistance();

            if (walkingSucced)
            {
                ROS_SUCCESS_LOG("Walk to distance SUCCESS");
                set_status(BT::SUCCESS);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::WalkToDistance::walkTowardsDistance()
{
     
    ROS_COLORED_LOG("dist to walk: %f", YELLOW, true, distance_to_walk);
    while (ros::ok())
    {
        refereeState= blackboard.getTarget("m_refereeStatus")->refereeStatus;  
        ros::Time curr_time_walk = ros::Time::now();
        ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
        
        if (dur_walk.toSec() == curr_time_walk.toSec())
        {
            prev_time_walk_ = curr_time_walk;
            return;
        }

        double delta_time_walk = dur_walk.toSec();
        prev_time_walk_ = curr_time_walk;
    
        double delta_distance = distance_to_walk - walked_distance;
        ROS_COLORED_LOG("walked dist: %f", ORANGE, false, walked_distance);
    
        if (walked_distance >= distance_to_walk || refereeState == 0)
        {
            stopWalking();
            if (reset_)
            {
                m_refereeInfo.refereeStatus = 0; 
                blackboard.setTarget("m_refereeStatus",m_refereeInfo);
            }
            walkingSucced = true;
            return;
        }
    
        double fb_move = 0.0, rl_angle = 0.0;
    
        calcFootstep(delta_distance, 0, delta_time_walk, fb_move, rl_angle);  // pan = 0
        ROS_COLORED_LOG("curr dist to ball: %f", CYAN, false, delta_distance);

        walked_distance += fabs(fb_move);
        accum_rotation += rl_angle;
        setWalkingParam(fb_move, 0, rl_angle, true);
        
        std_msgs::String command_msg;
        command_msg.data = "start";
        walk_command_pub.publish(command_msg);
        ros::Duration(0.1).sleep();
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::WalkToDistance::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("WalkToDistance HALTED: Stopped walking towards distance", "ORANGE", false, "Halted_walk_distance");
}
