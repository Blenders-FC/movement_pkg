#include "movement_pkg/tree_builder.h"
#include <string>

BT::ControlNode* BT::TreeBuilder::BuildTree()
{
    // Conditions
    auto* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
    auto* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
    auto* is_start_button = new BT::StartButtonCondition("IsStartButton");

    auto* ball_detected_condition = new BT::BallDetectedCondition("BallDetectedCondition");
    auto* ball_close_condition = new BT::BallCloseCondition("IsBallClose");
    
    // Actions
    auto* stand_up = new BT::StandUp("StandUp");
    //auto* search_ball = new BT::SearchBall("SearchBall");
    auto* left_long_kick_action = new  BT::LeftLongKick("LeftLongKick");
	auto* penalty_kick = new BT::PenaltyKick("PenaltyKick");

    // Control nodes
    BT::SequenceNodeWithMemory* root = new BT::SequenceNodeWithMemory("Root");
    BT::SequenceNodeWithMemory* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    BT::SequenceNodeWithMemory* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    BT::SequenceNodeWithMemory* ball_close_sequence = new BT::SequenceNodeWithMemory("BallCloseSequence");
    BT::SequenceNodeWithMemory* kick_sequence = new BT::SequenceNodeWithMemory("KickSequence");

    // Secuencia de inicio
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(stand_up);
    init_sequence->AddChild(is_start_button);

    // Search ballS
    ball_found_sequence->AddChild(ball_detected_condition);
    // does ball have to be centered? 


    // is distance from ball (ball area) big enough
    ball_close_sequence->AddChild(ball_close_condition);
    

    //if true then perform kick
    kick_sequence->AddChild(penalty_kick); // or left_long_kick_action? 

    //is that it? 


    // Construcción final
    root->AddChild(init_sequence);
    root->AddChild(ball_found_sequence);
    root->AddChild(ball_close_sequence);
    root->AddChild(kick_sequence);

    return root;
}
