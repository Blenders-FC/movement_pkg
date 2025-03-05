/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef GET_UP_BACKWARDS_ACTION_H
#define GET_UP_BACKWARDS_ACTION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <movement_pkg/nodes/utils.h>

namespace BT
{
class GetUpBackwards : public ActionNode, public utils
{
public:
    // Constructor
    explicit GetUpBackwards(std::string name);
    ~GetUpBackwards();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
};
}  // namespace BT

#endif  // GET_UP_BACKWARDS_ACTION_H
