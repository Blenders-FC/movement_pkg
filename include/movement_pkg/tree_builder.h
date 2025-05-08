/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TREE_BUILDER_H
#define TREE_BUILDER_H

#include <behavior_tree.h>

// Includes básicos de BT
#include "utils.h"

// Includes de condiciones
#include "nodes/manager_running_condition.h"
#include "nodes/manager_done_condition.h"
#include "nodes/start_button_condition.h"
#include "nodes/timer_condition.h"
#include "nodes/turn_left_action.h"
#include "nodes/turn_right_action.h"
#include "nodes/choose_kick_foot_condition.h"


// Includes de acciones
#include "nodes/stand_up_action.h"  // Añadido para StandUp
//#include "nodes/penalty_kick_action.h"     // Para PenaltyKick
#include "nodes/right_kick_action.h" //Para penalty v2
#include "nodes/left_kick_action.h" //Para penalty v2

namespace BT
{
    class TreeBuilder
    {
    public:
        static BT::ControlNode* BuildTree();  // Shared between runtime & visualization
    };
}

#endif // TREE_BUILDER_H