/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef QUADRANT_CONDITION_H
#define QUADRANT_CONDITION_H

#include "movement_pkg/utils.h"
#include "condition_node.h"


namespace BT
{
class QuadrantCondition : public ConditionNode, public virtual utils
{
    public:
        explicit QuadrantCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;
};
}  // namesapce BT

#endif // QUADRANT_CONDITION_H
