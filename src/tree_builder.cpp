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
	//auto* penalty_kick = new BT::PenaltyKick("PenaltyKick");
    auto* right_kick = new BT::RightKick("RightKick");
    auto* left_kick = new BT::LeftKick("LeftKick");
    auto* lower_head = new BT::MoveHead("MoveHead");
    auto* yaw_head = new BT::MoveHead("YawHead");
    auto* lower_head_2 = new BT::MoveHead("MoveHead2");
    auto* yaw_head_2 = new BT::MoveHead("YawHead2");
    auto* long_left_kick = new BT::LeftLongKick("LeftLongKick");

    // Control nodes
    BT::SequenceNodeWithMemory* root = new BT::SequenceNodeWithMemory("Root");
    BT::SequenceNodeWithMemory* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
    BT::SequenceNodeWithMemory* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
    BT::SequenceNodeWithMemory* ball_close_sequence = new BT::SequenceNodeWithMemory("BallCloseSequence");
    BT::SequenceNodeWithMemory* main_sequence = new BT::SequenceNodeWithMemory("MainSequence");
    BT::FallbackNode* ball_found_fallback = new BT::FallbackNode("BallFoundFallback");
    BT::FallbackNode* ball_close_fallback = new BT::FallbackNode("BallCloseFallback");
    BT::SequenceNodeWithMemory* kick_sequence = new BT::SequenceNodeWithMemory("KickSequence");
    BT::SequenceNodeWithMemory* lower_head_sequence = new BT::SequenceNodeWithMemory("MoveHeadSequence");
    BT::RepeatNode* repeat_node = new BT::RepeatNode("RepeatNode_"); // ?

    // Secuencia de inicio
    init_sequence->AddChild(is_manager_running);
    init_sequence->AddChild(is_manager_done);
    init_sequence->AddChild(stand_up);
    init_sequence->AddChild(lower_head);
    //init_sequence->AddChild(yaw_head);
    init_sequence->AddChild(is_start_button);

    //lower_head_sequence->AddChild(lower_head);

    // Search ballS
    //main_sequence->AddChild(ball_detected_condition);
    //ball_found_fallback->AddChild(ball_detected_condition);
    // does ball have to be centered? 


    // is distance from ball (ball area) big enough
    main_sequence->AddChild(ball_close_condition);
    //ball_close_fallback->AddChild(ball_close_condition);
    

    //if true then perform kick
    //main_sequence->AddChild(penalty_kick); // or left_long_kick_action? 
   // main_sequence->AddChild(left_kick);
   main_sequence->AddChild(left_kick);
    main_sequence->AddChild(lower_head_2);
    //main_sequence->AddChild(yaw_head_2);

    //is that it? 

    repeat_node->AddChild(main_sequence);


    // Construcción final
    root->AddChild(init_sequence);
    //root->AddChild(lower_head_sequence);
    root->AddChild(repeat_node);

    return root;
}
