/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TURN_SELECTOR_CONDITION_H
#define TURN_SELECTOR_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class TurnSelectorCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit TurnSelectorCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        double pan_angle_to_ball;
};
}  // namesapce BT

#endif // TURN_SELECTOR_CONDITION_H
