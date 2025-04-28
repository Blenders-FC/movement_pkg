/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TREE_BUILDER_H
#define TREE_BUILDER_H

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
#include "movement_pkg/walking_controller.h"
#include "movement_pkg/nodes/timer_condition.h"
#include "movement_pkg/nodes/turn_right_action.h"
#include "movement_pkg/nodes/simple_walk_action.h"
#include "movement_pkg/repeat_node.h"
#include "movement_pkg/nodes/robot_fallen_condition.h"
#include "movement_pkg/nodes/get_up_combined_action.h"
#include "movement_pkg/nodes/walk_to_point_action.h"


namespace BT
{
    class TreeBuilder
    {
    public:
        static BT::ControlNode* BuildTree();  // Shared between runtime & visualization
    };
}

#endif // TREE_BUILDER_H
