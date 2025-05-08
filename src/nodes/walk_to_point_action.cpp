/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_point_action.h"

int steps_cnt = 0;
BT::WalkToPoint::WalkToPoint(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToPoint::WaitForTick, this);
}

BT::WalkToPoint::~WalkToPoint() {}

void BT::WalkToPoint::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_walk_point");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_walk_point");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // set_status(BT::RUNNING);

            if (ball && std::isfinite(ball->distance) && std::isfinite(ball->pan_angle)
                && ball->distance > 0 && std::abs(ball->pan_angle) <= M_PI)
            {
                distance_to_ball = ball->distance;
                pan_angle_to_ball = ball->pan_angle;
                ROS_COLORED_LOG("Ball data loaded from blackboard!", GREEN, false);
            }
            else
            {
                ROS_COLORED_LOG("Invalid or missing ball data. Using default values", ORANGE, false);
            }

            this->setModule("walking_module");
            ROS_COLORED_LOG("Walking towards point in distance: %.4f and angle: %.4f", CYAN, true, distance_to_ball, pan_angle_to_ball);
            walkTowardsPoint();

            if (walkingSucced)
            {
                ROS_SUCCESS_LOG("Walking to point process has finished successfully!");
                std::cout << "steps_cnt: " << steps_cnt << std::endl;
                set_status(BT::SUCCESS);
            }
            else{
                ROS_ERROR_LOG("Walk to point FAILED", false);
                set_status(BT::FAILURE);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::WalkToPoint::walkTowardsPoint()
{
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
    
        if (distance_to_ball < 0)
        {
            distance_to_ball *= (-1);
        }
    
        double distance_to_kick = 0;// 0.30;  //0.22
        double distance_to_walk = distance_to_ball - distance_to_kick;
        std::cout << "walked_distance: " << walked_distance << std::endl;
    
        if (walked_distance >= distance_to_walk)
        {
            stopWalking();
            walkingSucced = true;
            return;
        }

        double delta_angle = pan_angle_to_ball - accum_rotation;

        // checking sign chance to avoid oscillation  ||  angle between a range of error
        if (delta_angle * prev_delta_angle < 0 || std::abs(delta_angle) < 0.01)
        {
            delta_angle = 0.0;  // Stop turning
        }

        prev_delta_angle = delta_angle;
    
        double fb_move = 0.0, rl_angle = 0.0;
    
        // std::cout << walked_distance << std::endl;
        // std::cout << distance_to_walk - walked_distance << std::endl;
        // std::cout << current_x_move_ << std::endl;
        // std::cout << delta_time_walk << std::endl;
    
        calcFootstep(distance_to_walk - walked_distance, delta_angle, delta_time_walk, fb_move, rl_angle);  // pan = 0
        walked_distance += fabs(fb_move);
        accum_rotation += rl_angle;
        std::cout << "fb_move: " << fb_move << std::endl;
        setWalkingParam(fb_move, 0, rl_angle, true);
        
        std_msgs::String command_msg;
        command_msg.data = "start";
        walk_command_pub.publish(command_msg);
        ros::Duration(0.1).sleep();
        steps_cnt++;
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::WalkToPoint::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("WalkToPoint HALTED: Stopped walking towards point", "ORANGE", false, "Halted_walk_point");
}
