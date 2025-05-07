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
    auto* ball_detected = new BT::BallDetectedCondition("BallDetected");
    auto* search_ball = new BT::SearchBall("SearchBall");
    auto* center_ball = new BT::CenterBallViolaJones("CenterBallViolaJones");
    auto* walk_to_target = new BT::WalkToTarget("WalkToTarget");
    auto* turn_right = new BT::TurnRight("TurnRight", 6);   // turning 90° (6 cycles of 15° each)
    auto* turn_left = new BT::TurnLeft("TurnLeft", 6);      // turning 90° (6 cycles of 15° each)
    auto* kick_selector = new BT::ChooseKickFootCondition("ChooseKickFootCondition");
    auto* left_kick = new BT::LeftKick("LeftKick");
    auto* right_kick = new BT::RightKick("RightKick");
    auto* is_fallen = new BT::RobotFallenCondition("IsFallen");
    auto* get_up = new BT::GetUpCombined("GetUp");
    // auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs

    
    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    auto* right_kick_seq = new BT::SequenceNodeWithMemory("RightKickSeq");
    auto* recovery_sequence = new BT::SequenceNodeWithMemory("RecoverySequence");
    auto* fallback_search_ball = new BT::FallbackNode("FallbackSearchBall");
    auto* fallback_kick_selector = new BT::FallbackNode("FallbackKickSelector");
    auto* reactive_fallback = new BT::FallbackNode("ReactiveFallback");
    auto* main_fallback = new BT::FallbackNode("MainFallback");
    auto* repeat_main_loop = new BT::RepeatNode("MainLoop");


    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    // Right Kick Sequence
    right_kick_seq->AddChild(kick_selector)
    right_kick_seq->AddChild(right_kick)

    // Kick fallback
    fallback_kick_selector->AddChild(right_kick_seq)
    fallback_kick_selector->AddChild(left_kick)

    // Walk to ball sequence
    ball_found_sequence->AddChild(ball_detected);
    ball_found_sequence->AddChild(center_ball);
    ball_found_sequence->AddChild(walk_to_target);
    ball_found_sequence->AddChild(fallback_kick_selector);

    // Search ball fallback
    fallback_search_ball->AddChild(search_ball);
    fallback_search_ball->AddChild(turn_right);

    // Add sequences to fallback
    main_fallback->AddChild(ball_found_sequence);
    main_fallback->AddChild(fallback_search_ball);

    // Recovery from fall sequence
    recovery_sequence->AddChild(is_fallen);
    recovery_sequence->AddChild(get_up);

    // Repeat main sequence
    reactive_fallback->AddChild(recovery_sequence);
    reactive_fallback->AddChild(main_fallback);
    repeat_main_loop->AddChild(reactive_fallback);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(repeat_main_loop);

    return root_node;
}
