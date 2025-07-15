/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/goals_detected_condition.h"


BT::GoalsDetectedCondition::GoalsDetectedCondition(const std::string &name) 
: BT::ConditionNode(name) {}

BT::ReturnStatus BT::GoalsDetectedCondition::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        goal_posts_ = getGoalsPositions();

        if (goal_posts_.empty())
        {
            ROS_COLORED_LOG("Received PointArray with no goal posts", RED, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
        else if (goal_posts_.size() != 2)
        {
            ROS_COLORED_LOG("Received %lu goal posts, flow needs 2", RED, false, goal_posts_.size());
            set_status(BT::FAILURE);
            return BT::FAILURE;
        }
        else if (goal_posts_.size() == 2)
        {
            ROS_SUCCESS_LOG("2 goal posts detected!");
            ROS_COLORED_LOG("First goal detected in pixels: x=%f y=%f", CYAN, false, goal_posts_[0].x, goal_posts_[0].y);
            ROS_COLORED_LOG("Second goal detected in pixels: x=%f y=%f", CYAN, false, goal_posts_[1].x, goal_posts_[1].y);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE; 
}
