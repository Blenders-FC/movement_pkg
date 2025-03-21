/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef BALL_DIRECTION_CONDITION_H
#define BALL_DIRECTION_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "movement_pkg/fov_calculation.h"
#include "condition_node.h"


namespace BT
{
class BallDirectionCondition : public ConditionNode, public CBDataManager, public FOVCalculation
{
    public:
        explicit BallDirectionCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
        const TargetInfo* ball = blackboard.getTarget("ball");
        double pan_angle_to_ball;
        double distance_to_ball;
};
}  // namesapce BT

#endif // BALL_DIRECTION_CONDITION_H
