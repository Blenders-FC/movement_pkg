/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/fov_walking_action.h"


BT::FOVWalking::FOVWalking(std::string name) 
    : ActionNode::ActionNode(name), 
    WalkingController(), 
    FOVCalculation()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&FOVWalking::WaitForTick, this);
}

BT::FOVWalking::~FOVWalking() {}

void BT::FOVWalking::WaitForTick()
{
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        set_status(BT::RUNNING);

        // Flow for walking to ball - using FOV calculation
        
        while (get_status() != BT::HALTED)
        {   
            this->setModule("walking_module");
            DEBUG_STDOUT(get_name() << "Walking towards target...");
            walkTowardsTarget();

            if (walkingSucced)
            {
                ROS_INFO_STREAM_COND(DEBUG_PRINT, GREEN_TEXT << "[SUCCESS] OP3 manager has reached the ball!" << RESET_TEXT);
                set_status(BT::SUCCESS);
            }
        }
    }
    ROS_ERROR_COND(DEBUG_PRINT, "ROS HAS STOPPED UNEXPECTEDLY IN BALL DIRECTION CONDITION");
    set_status(BT::FAILURE);
}

void BT::FOVWalking::walkTowardsTarget()
{
    ros::Time curr_time_walk = ros::Time::now();
    ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
    double delta_time_walk = dur_walk.nsec * 0.000000001 + dur_walk.sec;
    prev_time_walk_ = curr_time_walk;

    while (ros::ok())
    {
        if (distance_to_target < 0) distance_to_target *= (-1);
        std::cout << "dist to ball: " << distance_to_target << std::endl;
        
        if (distance_to_target > distance_to_kick_)
        {
            fb_move = 0.0;
            rl_angle = 0.0;
            distance_to_walk = distance_to_target - distance_to_kick_;

            calcFootstep(distance_to_walk, pan_angle, delta_time_walk, fb_move, rl_angle);
            setWalkingParam(fb_move, 0, rl_angle, true);

            walk_command.data = "start";
            walk_command_pub.publish(walk_command);

            ros::Duration(0.1).sleep();
        }
        else{
            stopWalking();
            walkingSucced = true;
            break;
        }
    }
}

void BT::FOVWalking::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    DEBUG_STDOUT("FOVWalking HALTED: Stopped walking.");
}
