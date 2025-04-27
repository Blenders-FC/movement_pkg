/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/tree_builder.h"

BT::ControlNode* BT::TreeBuilder::BuildTree()
{
    // Create Behavior Tree Nodes
    BT::ManagerRunningCondition* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
    BT::ManagerDoneCondition* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
    BT::StartButtonCondition* is_start_button = new BT::StartButtonCondition("IsStartButton");
    BT::StandUp* stand_up = new BT::StandUp("StandUp");
    BT::BallDetectedCondition* ball_detected = new BT::BallDetectedCondition("BallDetected");
    BT::SearchBall* search_ball = new BT::SearchBall("SearchBall");
    BT::BallDirectionCondition* ball_direction = new BT::BallDirectionCondition("BallDirection");

    // Create Control Nodes
    BT::SequenceNodeWithMemory* root_node = new BT::SequenceNodeWithMemory("RootNode");
    BT::SequenceNodeWithMemory* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    BT::SequenceNodeWithMemory* main_sequence = new BT::SequenceNodeWithMemory("MainLoop");
    BT::FallbackNode* fallback_node = new BT::FallbackNode("FallbackDecision");

    // Build the Behavior Tree
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(is_start_button);
    init_sequence->AddChild(stand_up);

    // Decision Making Structure
    BT::SequenceNodeWithMemory* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    ball_found_sequence->AddChild(ball_detected);
    ball_found_sequence->AddChild(ball_direction);

    // Add sequences to fallback
    fallback_node->AddChild(ball_found_sequence);
    fallback_node->AddChild(search_ball);

    // Attach the fallback node to the main sequence
    main_sequence->AddChild(fallback_node);

    // Root node sequence
    root_node->AddChild(init_sequence);
    root_node->AddChild(main_sequence);

    return root_node;
}
