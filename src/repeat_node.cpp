/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/repeat_node.h"

namespace BT
{

RepeatNode::RepeatNode(std::string name)
    : ControlNode::ControlNode(name) {}

BT::ReturnStatus RepeatNode::Tick()
{
    // This version only supports 1 child
    if (children_nodes_.size() != 1)
    {
        throw std::runtime_error("RepeatNode must have exactly one child.");
    }

    set_status(BT::RUNNING);

    unsigned int i = 0;

    auto* child = children_nodes_[0];
    BT::ReturnStatus child_status;

    if (child->get_type() == BT::ACTION_NODE)
    {
        child_status = child->get_status();
        if (child_status == BT::IDLE || child_status == BT::HALTED)
        {
            child->tick_engine.Tick();

            do
            {
                child_status = child->get_status();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (child_status != BT::RUNNING &&
                     child_status != BT::SUCCESS &&
                     child_status != BT::FAILURE);
        }
    }
    else
    {
        child_status = child->Tick();
    }

    switch (child_status)
    {
    case BT::SUCCESS:
        child->set_status(BT::IDLE);
        return BT::RUNNING;  // repeat again
    case BT::FAILURE:
        child->set_status(BT::IDLE);
        return BT::FAILURE;  // optionally repeat anyway if desired
    case BT::RUNNING:
        return BT::RUNNING;
    default:
        return BT::IDLE;
    }
}

void RepeatNode::Halt()
{
    if (!children_nodes_.empty())
    {
        children_nodes_[0]->Halt();
    }
    set_status(BT::IDLE);
}

int RepeatNode::DrawType()
{
    return BT::REPEAT;  // or create a new ID in the enum if needed
}

} // namespace BT
