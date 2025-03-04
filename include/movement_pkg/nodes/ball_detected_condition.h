/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef BALL_DETECTED_CONDITION_H
#define BALL_DETECTED_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"

namespace BT
{
class BallDetectedCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit BallDetectedCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
};
}  // namesapce BT

#endif // BALL_DETECTED_CONDITION_H
