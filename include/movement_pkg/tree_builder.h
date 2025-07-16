/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TREE_BUILDER_H
#define TREE_BUILDER_H

#include <behavior_tree.h>
#include "utils.h"
#include "movement_pkg/walking_controller.h"
#include "movement_pkg/repeat_node.h"

// Include custom nodes
#include "nodes/ball_detected_condition.h"
#include "nodes/ball_direction_condition.h"
#include "nodes/ball_in_center_condition.h"
#include "nodes/center_ball_Viola_Jones_action.h"
#include "nodes/center_ball_YOLO_CPU_action.h"
#include "nodes/center_ball_YOLO_Jetson_action.h"
#include "nodes/choose_kick_foot_condition.h"
#include "nodes/fov_walking_action.h"
#include "nodes/get_up_backwards_action.h"
#include "nodes/get_up_combined_action.h"
#include "nodes/get_up_forward_action.h"
#include "nodes/kick_side_decider_condition.h"
#include "nodes/left_kick_action.h"
#include "nodes/right_long_kick_action.h"
#include "nodes/manager_done_condition.h"
#include "nodes/manager_running_condition.h"
#include "nodes/online_walk_to_target_action.h"
#include "nodes/penalty_kick_action.h"
#include "nodes/right_kick_action.h"
#include "nodes/robot_fallen_condition.h"
#include "nodes/search_ball_action.h"
#include "nodes/send_head_to_home_action.h"
#include "nodes/simple_walk_action.h"
#include "nodes/stand_up_action.h"
#include "nodes/start_button_condition.h"
#include "nodes/timer_condition.h"
#include "nodes/turn_left_action.h"
#include "nodes/turn_right_action.h"
#include "nodes/walk_to_point_action.h"
#include "nodes/walk_to_target_action.h"
#include "nodes/can_move_condition.h"
#include "nodes/lateral_move_condition.h"
#include "nodes/right_move_condition.h"


namespace BT
{
    class TreeBuilder
    {
    public:
        static BT::ControlNode* BuildTree();  // Shared between runtime & visualization
    };
}

#endif // TREE_BUILDER_H
