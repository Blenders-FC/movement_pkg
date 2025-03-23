/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ball_direction_condition.h"


BT::BallDirectionCondition::BallDirectionCondition(const std::string &name) 
    : BT::ConditionNode(name),
    FOVCalculation()
    {}

BT::ReturnStatus BT::BallDirectionCondition::Tick()
{

    // Condition checking and state update

    while (ros::ok())
    {
        set_status(BT::RUNNING);
        ros::spinOnce();

        this->ball_center_position_ = getBallPosition();
        this->ball_position_x = ball_center_position_.x;
        this->ball_position_y = ball_center_position_.y;
        this->current_head_pan = getHeadPan();
        this->current_head_tilt = getHeadTilt();

        pan_angle_to_ball = ball->pan_angle;
        
        calcFocalLength_x();        // Updates focal length in x axis
        calcPanAngle();             // Updates pan_angle to the ball in body's frame

        if (std::abs(pan_angle_to_ball) <= 0.2618)  // 15°
        {
            calcFocalLength_y();        // Updates focal length in y axis
            calcDistanceToTarget();     // Updates distance to ball
            distance_to_ball = ball->distance;

            ROS_SUCCESS_LOG("OP3 manager is able to walk towards the ball!");
            std::cout << "dist: " << distance_to_target << "    pan: " << pan_angle << std::endl;
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

    ROS_ERROR_LOG("ROS stopped unexpectedly");
    return BT::FAILURE; 
}
