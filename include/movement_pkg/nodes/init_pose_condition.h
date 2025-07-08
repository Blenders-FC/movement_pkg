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
class InitPoseCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit InitPoseCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        bool is_valid_pose_;
};
}  // namesapce BT

#endif // BALL_DETECTED_CONDITION_H
