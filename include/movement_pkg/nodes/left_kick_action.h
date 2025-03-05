/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef LEFT_KICK_ACTION_H
#define LEFT_KICK_ACTION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <movemente_pkg/nodes/utils.h>

namespace BT
{
class LeftKickAction : public ActionNode, public utils
{
public:
    // Constructor
    explicit LeftKickAction(std::string name);
    ~LeftKickAction();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
};
}  // namespace BT

#endif  // LEFT_KICK_ACTION_H
