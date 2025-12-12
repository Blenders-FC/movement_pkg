/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef KICK_SIDE_DECIDER_CONDITION_H
#define KICK_SIDE_DECIDER_CONDITION_H

#include "movement_pkg/utils.h"
#include "condition_node.h"

namespace BT
{
class KickSideDeciderCondition : public ConditionNode, public virtual utils
{
    public:
        explicit KickSideDeciderCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        // instances for random
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_int_distribution<> dis;

        // side svariable
        int _side;
};
}  // namesapce BT

#endif // KICK_SIDE_DECIDER_CONDITION_H
