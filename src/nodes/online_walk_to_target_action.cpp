/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/online_walk_to_target_action.h"


BT::OnlineWalkToTarget::OnlineWalkToTarget(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&OnlineWalkToTarget::WaitForTick, this);
}

BT::OnlineWalkToTarget::~OnlineWalkToTarget() {}

void BT::OnlineWalkToTarget::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_online_walk");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_online_walk");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            set_status(BT::RUNNING);

            head_pan_angle_ = getHeadPan();
            head_tilt_angle_ = getHeadTilt();

            this->setModule("walking_module");
            ROS_TAGGED_ONCE_LOG("Walking towards target...", "PINK", true, "walk_towards_target");

            // Somewhere in your loop or behavior tree...
            bool result = walkToGoalPose(0.05, 0.0, 0.0);  // example goal

            if (result)
                ROS_SUCCESS_LOG("Walk to goal completed!");
            else
                ROS_ERROR_LOG("Walk to goal failed", false);
                set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

//////////////////////////////// Adapt em after tests ///////////////////////////////////
void BT::OnlineWalkToTarget::walkTowardsTarget(double head_pan_angle, double head_tilt_angle)
{
    ros::Time curr_time_walk = ros::Time::now();
    ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
    double delta_time_walk = dur_walk.nsec * 0.000000001 + dur_walk.sec;
    prev_time_walk_ = curr_time_walk;

    while (ros::ok())
    {
        double distance_to_target = calculateDistance(head_tilt_angle);
        if (distance_to_target < 0) distance_to_target *= (-1);
        ROS_COLORED_LOG("dist to ball: ", CYAN, false, distance_to_target);
        
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
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
}

double BT::OnlineWalkToTarget::calculateDistance(double head_tilt)
{
    double distance = CAMERA_HEIGHT_ * tan(M_PI * 0.5 + head_tilt - hip_pitch_offset_);
    return distance;
}
////////////////////////////////////////////////////////////////////////////////////////

void BT::OnlineWalkToTarget::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("OnlineWalkToTarget HALTED: Stopped online walking", "ORANGE", false, "Halted_online_walking");
}
