/*
    Authors:
        victor Gil
*/

#ifndef BALL_CLOSE_CONDITION_H
#define BALL_CLOSE_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class BallCloseCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit BallCloseCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_area_position_;
        double ballArea;
};
}  // namesapce BT

#endif // BALL_CLOSE_CONDITION_H
