/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef BALL_IN_CENTER_CONDITION_H
#define BALL_IN_CENTER_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"

namespace BT
{
class BallInCenterCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit BallInCenterCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
        double xerror_ = 0;
        double yerror_ = 0;
};
}  // namesapce BT

#endif // BALL_IN_CENTER_CONDITION_H
