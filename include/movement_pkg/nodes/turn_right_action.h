/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TURN_RIGHT_ACTION_H
#define TURN_RIGHT_ACTION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <movemente_pkg/nodes/utils.h>

namespace BT
{
class TurnRight : public ActionNode, public utils
{
    public:
        // Constructor
        explicit TurnRight(std::string name);
        ~TurnRight();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
    private:
        void turn();
};
}  // namespace BT

#endif  // TURN_RIGHT_ACTION_H
