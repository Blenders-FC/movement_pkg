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
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        set_status(BT::RUNNING);

        head_pan_angle_ = getHeadPan();
        head_tilt_angle_ = getHeadTilt();

        this->setModule("walking_module");
        DEBUG_STDOUT(get_name() << "Walking towards target...");
        walkTowardsTarget(head_pan_angle_, head_tilt_angle_);

        if (walkingSucced)
        {
            DEBUG_STDOUT(get_name() << "Walk to target SUCCESS");
            set_status(BT::SUCCESS);
        }
    }
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
        std::cout << "dist to ball: " << distance_to_target << std::endl;
        
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
    DEBUG_STDOUT("WalkToTarget HALTED: Stopped walking.");
}
