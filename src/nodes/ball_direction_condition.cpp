/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ball_direction_condition.h"


BT::BallDirectionCondition::BallDirectionCondition(const std::string &name) 
    : BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallDirectionCondition::Tick()
{

    // Condition checking and state update

    while (ros::ok())
    {
        // set_status(BT::RUNNING);

        pan_angle_to_ball = getHeadPan();
        ROS_COLORED_LOG("pan angle: %f", PINK, false, pan_angle_to_ball);

        if (std::abs(pan_angle_to_ball) <= 0.2618)  // 15°
        {
            ROS_SUCCESS_LOG("OP3 manager is able to walk towards the ball!");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            ROS_COLORED_LOG("OP3 is NOT able to walk towards the ball! Turning...", YELLOW, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
