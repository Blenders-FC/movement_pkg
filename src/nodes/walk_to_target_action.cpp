/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_target_action.h"


BT::WalkToTarget::WalkToTarget(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToTarget::WaitForTick, this);
}

BT::WalkToTarget::~WalkToTarget() {}

void BT::WalkToTarget::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        set_status(BT::RUNNING);

        // Perform action...
        IF (get_status() != BT::HALTED)
        {
            head_pan_angle_ = getHeadPan();
            head_tilt_angle_ = getHeadTilt();

            this->setModule("walking_module");
            ROS_TAGGED_ONCE_LOG("Walking towards target...");
            walkTowardsTarget(head_pan_angle_, head_tilt_angle_);

            if (walkingSucced)
            {
                ROS_SUCCESS_LOG("Walk to target SUCCESS");
                set_status(BT::SUCCESS);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE;
}

void BT::WalkToTarget::walkTowardsTarget(double head_pan_angle, double head_tilt_angle)
{
    ros::Time curr_time_walk = ros::Time::now();
    ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
    double delta_time_walk = dur_walk.nsec * 0.000000001 + dur_walk.sec;
    prev_time_walk_ = curr_time_walk;

    while (ros::ok())
    {
        double distance_to_target = calculateDistance(head_tilt_angle);
        if (distance_to_target < 0) distance_to_target *= (-1);
        ROS_COLORED_LOG("dist to ball: ", distance_to_target, CYAN, false);
        
        if (distance_to_target > distance_to_kick_)
        {
            fb_move = 0.0;
            rl_angle = 0.0;
            distance_to_walk = distance_to_target - distance_to_kick_;

            calcFootstep(distance_to_walk, head_pan_angle, delta_time_walk, fb_move, rl_angle);
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
    ROS_ERROR_LOG("ROS stopped unexpectedly");
}

double BT::WalkToTarget::calculateDistance(double head_tilt)
{
    double distance = CAMERA_HEIGHT_ * tan(M_PI * 0.5 + head_tilt - hip_pitch_offset_);
    return distance;
}

void BT::WalkToTarget::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("WalkToTarget HALTED: Stopped walking.");
}
