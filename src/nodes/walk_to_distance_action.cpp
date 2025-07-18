/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_distance_action.h"


BT::WalkToDistance::WalkToDistance(std::string name, double distance) 
: ActionNode::ActionNode(name), WalkingController(), distance_to_walk(distance)
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
        ros::Time curr_time_walk = ros::Time::now();
        ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
        
        if (dur_walk.toSec() == curr_time_walk.toSec())
        {
            prev_time_walk_ = curr_time_walk;
            return;
        }

        double delta_time_walk = dur_walk.toSec();
        prev_time_walk_ = curr_time_walk;
    
        // if (distance_to_ball < 0)
        // {
        //     distance_to_ball *= (-1);
        // }
    
        // double distance_to_walk = distance_to_ball - distance_to_kick_;
        double delta_distance = distance_to_walk - walked_distance;
        ROS_COLORED_LOG("walked dist: %f", ORANGE, false, walked_distance);
    
        if (walked_distance >= distance_to_walk)
        {
            stopWalking();
            walkingSucced = true;
            return;
        }
    
        double fb_move = 0.0, rl_angle = 0.0;
    
        // std::cout << walked_distance << std::endl;
        // std::cout << distance_to_walk - walked_distance << std::endl;
        // std::cout << current_x_move_ << std::endl;
        // std::cout << delta_time_walk << std::endl;
    
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
