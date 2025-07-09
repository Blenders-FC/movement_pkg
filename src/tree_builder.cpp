/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/tree_builder.h"

BT::ControlNode* BT::TreeBuilder::BuildTree()
{
    // Create leaves
    auto* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
    auto* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
    auto* is_start_button = new BT::StartButtonCondition("IsStartButton");
    auto* stand_up = new BT::StandUp("StandUp");
    auto* simple_walk = new BT::SimpleWalk("SimpleWalk");
    auto* simple_walk_2 = new BT::SimpleWalk("SimpleWalk2");
    auto* simple_walk_3 = new BT::SimpleWalk("SimpleWalk3");
    auto* is_movement_able = new BT::CanMoveCondition("CanMoveCondition");
    auto* is_lateral_movement_able = new BT::LateralMoveCondition("LateralMoveCondition");
    auto* is_right_movement_able = new BT::RightMoveCondition("RightMoveCondition");
    auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs

    
    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* moving_seq = new BT::SequenceNodeWithMemory("MovSequence");
    auto* first_walking_seq = new BT::SequenceNodeWithMemory("WalkSequence");
    auto* second_walking_seq = new BT::SequenceNodeWithMemory("WalkSequencePt2");
    auto* parallel_recovery = new BT::ParallelNode("ParallelRecovery", 2); // success_threshold=2
    auto* main_fallback = new BT::FallbackNode("MainFallback");
    auto* lateral_fallback = new BT::FallbackNode("LateralFallback");
    auto* right_fallback = new BT::FallbackNode("RightFallback");
    auto* repeat_main_loop = new BT::RepeatNode("MainLoop");


    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    //Walk forward
    parallel_recovery ->AddChild(simple_walk);//simple walk ir derecho
    parallel_recovery ->AddChild(timer_condition);
    //Walking throught
    main_fallback ->AddChild(moving_seq);
    moving_seq ->AddChild(is_movement_able);
    moving_seq ->AddChild(lateral_fallback);
    moving_seq ->AddChild(parallel_recovery);

    lateral_fallback ->AddChild(first_walking_seq);
    first_walking_seq ->AddChild(is_lateral_movement_able);
    first_walking_seq ->AddChild(right_fallback);
    first_walking_seq ->AddChild(simple_walk3);//simple walk ir la izquierda

    right_fallback ->AddChild(second_walking_seq);
    second_walking_seq ->AddChild(is_right_movement_able);
    second_walking_seq ->AddChild(simple_walk_2);//simple walk ir a la derecha
    
    // Repeat main sequence
    repeat_main_loop->AddChild(main_fallback);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(repeat_main_loop);

    return root_node;
}
