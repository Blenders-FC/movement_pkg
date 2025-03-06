/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CHOOSE_FOOT_KICK_CONDITION_H
#define CHOOSE_FOOT_KICK_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"

namespace BT
{
class ChooseKickFootCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit ChooseKickFootCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        double head_pan_;
};
}  // namesapce BT

#endif // CHOOSE_FOOT_KICK_CONDITION_H
