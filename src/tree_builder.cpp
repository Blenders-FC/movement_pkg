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
    auto* ball_detected = new BT::BallDetectedCondition("BallDetected");
    auto* search_ball = new BT::SearchBall("SearchBall");
    auto* center_ball = new BT::CenterBallViolaJones("CenterBallViolaJones");
    auto* walk_to_point = new BT::WalkToPoint("WalkToPoint");

    // Create Control Nodes
    auto* root_node = new BT::SequenceNodeWithMemory("RootNode");
    auto* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    auto* main_sequence = new BT::SequenceNodeWithMemory("MainLoop");
    auto* fallback_node = new BT::FallbackNode("FallbackDecision");

    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    // Decision Making Structure
    auto* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    ball_found_sequence->AddChild(ball_detected);
    ball_found_sequence->AddChild(ball_direction);

    // Add sequences to fallback
    fallback_node->AddChild(ball_found_sequence);
    fallback_node->AddChild(search_ball);

    // Attach the fallback node to the main sequence
    main_sequence->AddChild(fallback_node);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(walk_to_point);

    return root_node;
}
