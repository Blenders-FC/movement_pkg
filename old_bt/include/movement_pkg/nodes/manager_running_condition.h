/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef MANAGER_RUNNING_CONDITION_H
#define MANAGER_RUNNING_CONDITION_H

#include "movement_pkg/utils.h"
#include "condition_node.h"


namespace BT
{
class ManagerRunningCondition : public ConditionNode, public virtual utils
{
    public:
        explicit ManagerRunningCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        bool checkManagerRunning(std::string& manager_name);

        std::string manager_name = "/op3_manager";
        bool already_logged_ = false;
};
}  // namesapce BT

#endif  // MANAGER_RUNNING_CONDITION_H
