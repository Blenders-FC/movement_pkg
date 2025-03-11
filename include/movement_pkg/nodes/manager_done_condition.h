/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef MANAGER_DONE_CONDITION_H
#define MANAGER_DONE_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class ManagerDoneCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit ManagerDoneCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        bool DEBUG_PRINT_ = false;
};
}  // namesapce BT

#endif  // MANAGER_DONE_CONDITION_H
