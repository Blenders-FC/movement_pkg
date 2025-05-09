#include "movement_pkg/tree_builder.h"
#include <string>

BT::ControlNode* BT::TreeBuilder::BuildTree()
{
    // Condiciones
    BT::ManagerRunningCondition* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
    BT::ManagerDoneCondition* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
    BT::StartButtonCondition* is_start_button = new BT::StartButtonCondition("IsStartButton");
    BT::TimerCondition* is_timer_condition_done = new BT::TimerCondition("TimerCondition",5);
    
    // Acciones
    BT::StandUp* stand_up = new BT::StandUp("StandUp");
    BT::TurnRight* turn_right = new BT::TurnRight("TurnRight",1);
    BT::TurnLeft* turn_left = new BT::TurnLeft("TurnLeft",1);
    BT::KickSideDeciderCondition* kick_side_decider_condition = new BT::KickSideDeciderCondition("KickSideDeciderCondition");
    BT::LeftLongKick* left_long_kick_action = new  BT::LeftLongKick("LeftLongKick");
    BT::RightLongKick* right_long_kick_action = new  BT::RightLongKick("RightLongKick");


    // Nodos de control
    BT::SequenceNodeWithMemory* root = new BT::SequenceNodeWithMemory("Root");
    BT::SequenceNodeWithMemory* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    BT::FallbackNode* random_choice = new BT::FallbackNode("RandomChoice");
    BT::SequenceNodeWithMemory* right_sequence = new BT::SequenceNodeWithMemory("RightSequence");
    BT::SequenceNodeWithMemory* left_sequence = new BT::SequenceNodeWithMemory("LeftSequence");

    // Secuencia de inicio
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(stand_up);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(is_timer_condition_done);

    // Secuencia derecha
    right_sequence->AddChild(turn_right);
    //right_sequence->AddChild(left_kick);
    right_sequence->AddChild(left_long_kick_action);

    // Secuencia izquierda
    left_sequence->AddChild(kick_side_decider_condition);
    left_sequence->AddChild(turn_left);
    //left_sequence->AddChild(right_kick);
    left_sequence->AddChild(right_long_kick_action);

    // Elección aleatoria (implementada como Fallback con condiciones)
    random_choice->AddChild(left_sequence);   // Si falla, va a izquierda
    random_choice->AddChild(right_sequence);  // Primero intenta derecha

    // Construcción final
    root->AddChild(init_sequence);
    root->AddChild(random_choice);

    return root;
}