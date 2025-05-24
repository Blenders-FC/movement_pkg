/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef SIMPLE_WALK_ACTION_H
#define SIMPLE_WALK_ACTION_H

#include "movement_pkg/walking_controller.h"
#include <action_node.h>


namespace BT
{
class SimpleWalk : public ActionNode, public WalkingController
{
    public:
        explicit SimpleWalk(std::string name);  // Constructor
        ~SimpleWalk();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
    
    private:
        std::string walking_command_;
};
}  // namespace BT

#endif  // SIMPLE_WALK_ACTION_H
