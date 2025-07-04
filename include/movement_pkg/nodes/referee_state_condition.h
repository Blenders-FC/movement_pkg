/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef REFEREE_STATE_CONDITION_H
#define REFEREE_STATE_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class RefereeStateCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit RefereeStateCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
};
}  // namesapce BT

#endif // RefereeStateCondition
