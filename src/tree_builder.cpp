/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/tree_builder.h"

BT::ControlNode* BT::TreeBuilder::BuildTree()
{
    // Create Behavior Tree Nodes
    auto* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
    auto* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
    auto* is_start_button = new BT::StartButtonCondition("IsStartButton");
    auto* stand_up = new BT::StandUp("StandUp");
    auto* stop_walking = new BT::StandUp("StopWalking");
    auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs
    auto* walk_node = new BT::SimpleWalk("SimpleWalk");
    auto* turn_right = new BT::TurnRight("TurnRight", 6);  // turning 90° (6 cycles of 15° each)
    auto* is_fallen = new BT::RobotFallenCondition("IsFallen");
    auto* get_up = new BT::GetUpCombined("GetUp");

    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* main_sequence = new BT::SequenceNodeWithMemory("MainLoop");
    auto* parallel_walk_timer = new BT::ParallelNode("ParallelWalkTimer", 2); // success_threshold=1, failure_threshold=1
    auto* repeat_main_loop = new BT::RepeatNode("MainLoopLoop");
    auto* reactive_fallback = new BT::FallbackNode("ReactiveFallback");  // acts like ReactiveFallback
    auto* recovery_sequence = new BT::SequenceNodeWithMemory("RecoverySequence");


    // Recovery from fall sequence
    recovery_sequence->AddChild(is_fallen);
    recovery_sequence->AddChild(get_up);
    reactive_fallback->AddChild(recovery_sequence);

    // Parallel node: walk and time concurrently
    parallel_walk_timer->AddChild(walk_node);
    parallel_walk_timer->AddChild(timer_condition);

    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    // Attach the fallback node to the main sequence
    main_sequence->AddChild(parallel_walk_timer);
    main_sequence->AddChild(stop_walking);
    main_sequence->AddChild(turn_right);

    // Repeat main sequence
    reactive_fallback->AddChild(main_sequence);
    repeat_main_loop->AddChild(reactive_fallback);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(repeat_main_loop);

    return root_node;
}
