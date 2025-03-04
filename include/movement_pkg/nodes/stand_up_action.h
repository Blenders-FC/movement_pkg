/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_TARGET_ACTION_H
#define WALKING_TO_TARGET_ACTION_H

#include "movement_pkg/walking_controller.h"
#include <action_node.h>


namespace BT
{
class WalkToTarget : public ActionNode, public WalkingController
{
    public:
        // Constructor
        explicit WalkToTarget(std::string name);
        ~WalkToTarget();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
};
}  // namespace BT

#endif  // WALKING_TO_TARGET_ACTION_H
