/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef MOTION_CONTROLLER_NODE_H
#define MOTION_CONTROLLER_NODE_H

#include <behavior_tree.h>

// Include custom nodes
#include "utils.h"
#include "nodes/ball_detected_condition.h"
#include "nodes/walk_to_target_action.h"
#include "nodes/manager_running_condition.h"

namespace BT
{
class MotionControllerNode 
{
    public:
        MotionControllerNode();
        void run();

    private:
        BT::SequenceNodeWithMemory* root_node_;
};
}

#endif // MOTION_CONTROLLER_NODE_H
