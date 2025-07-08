/*
    Authors:
        Victor Gil
*/

#include "movement_pkg/nodes/ball_close_condition.h"


BT::BallCloseCondition::BallCloseCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::BallCloseCondition::Tick()
{
    // Condition checking and state update
    
    while(ros::ok())
    {
        
        ball = getBallArea();
        ROS_COLORED_LOG("Ball y: %f",YELLOW, false,  ball.y);
        ROS_COLORED_LOG("Ball area: %f",YELLOW, false,  ball.z);
        while (ball.y > 998.0 || ball.z < 3000)
        {

            //ROS_COLORED_LOG("Ball NOT CLOSE enough!", YELLOW, false);
            ball = getBallArea();
        }
        ROS_SUCCESS_LOG("Ball is CLOSE");
        ROS_COLORED_LOG("Ball y: %f",YELLOW, false,  ball.y);
        ROS_COLORED_LOG("Ball area: %f",YELLOW, false,  ball.z);
        set_status(BT::SUCCESS);
        return BT::SUCCESS;

    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
