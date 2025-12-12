/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef RIGHT_KICK_ACTION_H
#define RIGHT_KICK_ACTION_H

#include <movement_pkg/utils.h>
#include <action_node.h>


namespace BT
{
class RightKick : public ActionNode, public virtual utils
{
public:
    // Constructor
    explicit RightKick(std::string name);
    ~RightKick();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
};
}  // namespace BT

#endif  // RIGHT_KICK_ACTION_H
