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
#include "nodes/manager_done_condition.h"
#include "nodes/start_button_condition.h"

namespace BT
{
class MotionControllerNode 
{
};
}

#endif // MOTION_CONTROLLER_NODE_H
