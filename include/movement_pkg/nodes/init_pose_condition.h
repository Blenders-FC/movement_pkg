/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef INIT_POSE_CONDITION_H
#define INIT_POSE_CONDITION_H

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

#endif // INIT_POSE_CONDITION_H
