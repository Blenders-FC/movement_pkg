/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef GET_UP_FORWARD_ACTION_H
#define GET_UP_FORWARD_ACTION_H

#include <movement_pkg/nodes/utils.h>
#include <action_node.h>


namespace BT
{
class GetUpForward : public ActionNode, public utils
{
public:
    // Constructor
    explicit GetUpForward(std::string name);
    ~GetUpForward();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
};
}  // namespace BT

#endif  // GET_UP_FORWARD_ACTION_H
