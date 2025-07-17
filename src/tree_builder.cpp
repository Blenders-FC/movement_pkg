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
    auto* head_to_home = new BT::HeadToHome("HeadToHome");
    auto* walk_to_ball = new BT::WalkToBallPosition("WalkToBallPosition");
    auto* init_position = new BT::InitPoseCondition("InitPoseCondition");
    auto* center_goal = new BT::CenterGoalYOLOPID("CenterGoalYOLOPID");
    auto* quadrant_condition = new BT::QuadrantCondition("QuadrantCondition");
    auto* right_search = new BT::RightSearchGoal("RightSearchGoal");
    auto* left_search = new BT::LeftSearchGoal("LeftSearchGoal");
    auto* goals_detector_r = new BT::GoalsDetectedCondition("RightGoalsDetectedCondition");
    auto* goals_detector_l = new BT::GoalsDetectedCondition("LeftGoalsDetectedCondition");
    auto* center_goal_slow = new BT::CenterGoalSlow("CenterGoalSlow");
    auto* walk_to_dist = new BT::WalkToDistance("WalkToDistance", 5);    // walking dist (m)
    auto* turn_right_entry = new BT::TurnRight("TurnRightEntry", 6);   // turning 90° (6 cycles of 15° each)
    // auto* timer_condition = new BT::TimerCondition("TimerCondition", 5.0);  // 5 secs

    
    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    auto* right_kick_seq = new BT::SequenceNodeWithMemory("RightKickSeq");
    auto* turning_head_home_seq = new BT::SequenceNodeWithMemory("TurningAndHeadToHome");
    auto* fallback_search_ball = new BT::FallbackNode("FallbackSearchBall");
    auto* fallback_kick_selector = new BT::FallbackNode("FallbackKickSelector");
    auto* reactive_fallback = new BT::FallbackNode("ReactiveFallback");
    auto* parallel_recovery = new BT::ParallelNode("ParallelRecovery", 2); // success_threshold=2
    auto* main_fallback = new BT::FallbackNode("MainFallback");
    auto* calc_init_position = new BT::SequenceNodeWithMemory("calcInitPosition");
    auto* left_search_seq = new BT::SequenceNode("leftSearchSeq");
    auto* init_position_fb = new BT::FallbackNode("isInitPosition");
    auto* repeat_main_loop = new BT::RepeatNode("MainLoop");
    auto* search_side_selector = new BT::FallbackNode("SearchSideSelector");
    auto* prl_left_search = new BT::ParallelNode("ParallelLeftSearch", 2); // success_threshold=2
    auto* prl_right_search = new BT::ParallelNode("ParallelRightSearch", 2); // success_threshold=2
    auto* entry_rutine = new BT::SequenceNodeWithMemory("entryRutine");

    // Entry rutine
    entry_rutine->AddChild(walk_to_dist);
    entry_rutine->AddChild(turn_right_entry);

    // calc init position
    prl_right_search->AddChild(right_search);
    prl_right_search->AddChild(goals_detector_r);
    prl_left_search->AddChild(left_search);
    prl_left_search->AddChild(goals_detector_l);

    left_search_seq->AddChild(quadrant_condition);
    left_search_seq->AddChild(prl_left_search);

    search_side_selector->AddChild(left_search_seq);
    search_side_selector->AddChild(prl_right_search);

    calc_init_position->AddChild(search_side_selector);
    //calc_init_position->AddChild(center_goal);

    init_position_fb->AddChild(init_position);
    init_position_fb->AddChild(calc_init_position);

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

    // Turning and Head to Home sequence
    turning_head_home_seq->AddChild(turn_right);
    turning_head_home_seq->AddChild(head_to_home);

    // Search ball fallback
    fallback_search_ball->AddChild(search_ball);
    fallback_search_ball->AddChild(turning_head_home_seq);

    // Add sequences to fallback
    main_fallback->AddChild(ball_found_sequence);
    main_fallback->AddChild(fallback_search_ball);

    // Repeat main sequence
    repeat_main_loop->AddChild(init_position_fb);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(entry_rutine);

    return root_node;
}
