/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef REPEAT_NODE_H
#define REPEAT_NODE_H

#include <control_node.h>
#include <string>

namespace BT
{
class RepeatNode : public ControlNode
{
public:
    RepeatNode(std::string name);
    ~RepeatNode();

    BT::ReturnStatus Tick();
    void Halt();
    int DrawType();
};
} // namespace BT

#endif // REPEAT_NODE_H
