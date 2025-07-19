/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALK_MIDDLE_FIELD_CONDITION_H
#define WALK_MIDDLE_FIELD_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class WalkMiddleFieldCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit WalkMiddleFieldCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;
};
}  // namesapce BT

#endif // WALK_MIDDLE_FIELD_CONDITION_H
