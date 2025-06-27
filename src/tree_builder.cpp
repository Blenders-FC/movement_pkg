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
    auto* left_kick_m = new BT::LeftKick("LeftKick");
    auto* right_kick = new BT::RightKick("RightKick");
    auto* head_to_home = new BT::HeadToHome("HeadToHome");
    auto* head_to_home_reset = new BT::HeadToHomeReset("HeadToHomeReset");
    auto* turn_n_times = new BT::RepeatNTimes("RepeatNTimes");
    //auto* walk_n_times  = new BT::RepeatNTimes("RepeatNTimes");
    auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs
    // auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs

    //walking node TESTI
    auto* simple_walk_action = new BT::SimpleWalk("SimpleWalk");

    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    auto* right_kick_seq = new BT::SequenceNodeWithMemory("RightKickSeq");
    auto* turning_head_home_seq = new BT::SequenceNodeWithMemory("TurningAndHeadToHome");
    auto* walk_head_home_seq = new BT::SequenceNodeWithMemory("WalkAndHeadToHome");
    auto* fallback_search_ball = new BT::FallbackNode("FallbackSearchBall");
    auto* fallback_kick_selector = new BT::FallbackNode("FallbackKickSelector");
    auto* reactive_fallback = new BT::FallbackNode("ReactiveFallback");
    auto* parallel_recovery = new BT::ParallelNode("ParallelRecovery", 2); // success_threshold=2
    auto* parallel_walk_timer = new BT::ParallelNode("ParallelWalkTimer", 2); // success_threshold=1, failure_threshold=1
    auto* fallback_turns = new BT::FallbackNode("Fallback");
    auto* main_fallback = new BT::FallbackNode("MainFallback");
    auto* repeat_main_loop = new BT::RepeatNode("MainLoop");


    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    // Right Kick Sequence
    right_kick_seq->AddChild(kick_selector);
    right_kick_seq->AddChild(right_kick);

    // Kick fallback
    fallback_kick_selector->AddChild(right_kick_seq);
    fallback_kick_selector->AddChild(left_kick);

    // Walk to ball sequence
    ball_found_sequence->AddChild(ball_detected);
    ball_found_sequence->AddChild(center_ball);
    ball_found_sequence->AddChild(walk_to_target);
    ball_found_sequence->AddChild(fallback_kick_selector);

    //Turn 5 times sequence
    //repeat_turn_right->AddChild(turn_right);

    // Turning and Head to Home sequence
    turning_head_home_seq->AddChild(turn_n_times);
    turning_head_home_seq->AddChild(turn_right);
    //turning_head_home_seq->AddChild(repeat_turn_right);
    turning_head_home_seq->AddChild(head_to_home);

    // Parallel node: walk and time concurrently
    parallel_walk_timer->AddChild(simple_walk_action);
    parallel_walk_timer->AddChild(timer_condition);

    // Walking and Head to Home sequence
    // walk_head_home_seq->AddChild(left_kick_m); TESTI
    //walk_head_home_seq->AddChild(walk_n_times);
    walk_head_home_seq->AddChild(parallel_walk_timer);
    walk_head_home_seq->AddChild(head_to_home_reset);

    // Search ball fallback
    fallback_search_ball->AddChild(search_ball);
    fallback_search_ball->AddChild(fallback_turns);
    //fallback_search_ball->AddChild(turning_head_home_seq);

    //add sequence to fallback turns
    fallback_turns->AddChild(turning_head_home_seq);
    fallback_turns->AddChild(walk_head_home_seq);


    // Add sequences to fallback
    main_fallback->AddChild(ball_found_sequence);
    main_fallback->AddChild(fallback_search_ball);

    // Repeat main sequence
    repeat_main_loop->AddChild(main_fallback);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(repeat_main_loop);

    return root_node;
}
