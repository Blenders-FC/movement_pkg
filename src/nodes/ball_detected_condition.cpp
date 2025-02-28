/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movemente_pkg/nodes/ball_detected_condition.h"


BallDetectedCondition(const std::string &name) : ConditionNode::ConditionNode(name) {
    CBDataManager& dataManager = CBDataManager::getInstance();
}

BT::ReturnStatus BT::BallDetectedCondition::Tick()
{
    // Condition checking and state update

    ball_center_position_ = dataManager.getBallPosition();
    
    if (ball_center_position_.x != 999 || ball_center_position_.y != 999)
    {
        set_status(BT::SUCCESS);
        return BT::SUCCESS;
    }
    else
    {
        set_status(BT::FAILURE);
        return BT::FAILURE;
    }
}
