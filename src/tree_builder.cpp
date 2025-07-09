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
    auto* walk_to_target = new BT::WalkToTarget("WalkToTarget");
    auto* simple_walk = new BT::SimpleWalk("SimpleWalk");
    // auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs

    
    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* walking_seq = new BT::SequenceNodeWithMemory("WalkSequence");
    auto* parallel_recovery = new BT::ParallelNode("ParallelRecovery", 2); // success_threshold=2
    auto* main_fallback = new BT::FallbackNode("MainFallback");
    auto* repeat_main_loop = new BT::RepeatNode("MainLoop");


    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    //Walking throught
    walking_seq ->AddChild(simple_walk);
    walking_seq ->AddChild(walk_to_target);
    // Add sequences to fallback
    main_fallback->AddChild(walking_seq);
    // Repeat main sequence
    repeat_main_loop->AddChild(main_fallback);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(repeat_main_loop);

    return root_node;
}
