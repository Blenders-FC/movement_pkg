/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef STAND_UP_ACTION_H
#define STAND_UP_ACTION_H

#include "movement_pkg/walking_controller.h"
#include <action_node.h>


namespace BT
{
class StandUp : public ActionNode, public WalkingController
{
    public:
        // Constructor
        explicit StandUp(std::string name);
        ~StandUp();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
};
}  // namespace BT

#endif  // STAND_UP_ACTION_H
