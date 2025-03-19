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
#include "nodes/manager_running_condition.h"
#include "nodes/manager_done_condition.h"
#include "nodes/start_button_condition.h"
#include "nodes/stand_up_action.h"
#include "nodes/ball_detected_condition.h"
#include "movement_pkg/nodes/search_ball_action.h"
#include "movement_pkg/nodes/ball_direction_condition.h"


namespace BT
{
class MotionControllerNode 
{
};
}

#endif // MOTION_CONTROLLER_NODE_H
