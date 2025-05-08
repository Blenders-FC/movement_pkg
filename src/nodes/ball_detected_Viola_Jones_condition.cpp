/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/ball_detected_Viola_Jones_condition.h"


BT::BallDetectedViolaJonesCondition::BallDetectedViolaJonesCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallDetectedViolaJonesCondition::Tick()
{
    while (ros::ok())
    {
        ball_center_position_ = getBallPosition();

        ball_detected = (ball_center_position_.x != 999 && ball_center_position_.x != 0) ||
                             (ball_center_position_.y != 999 && ball_center_position_.y != 0);

        if (ball_detected)
        {
            no_detection_counter = 0;  // Reset counter
            ROS_SUCCESS_LOG("BALL detected!");
            ROS_COLORED_LOG("Ball detected with positions: x=%f y=%f", CYAN, false, ball_center_position_.x, ball_center_position_.y);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
            no_detection_counter++;
            if (no_detection_counter > max_no_detection_count)
            {
                ROS_COLORED_LOG("BALL NOT detected", RED, false);
                set_status(BT::FAILURE);
                return BT::FAILURE;
            }
            else
            {
                // Mantener último estado (SUCCESS) temporalmente por cortafuegos
                ROS_COLORED_LOG("Ball temporarily lost, waiting to confirm...", ORANGE, false);
            }
        }
    }

    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
