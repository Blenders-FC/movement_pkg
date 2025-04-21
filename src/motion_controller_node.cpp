/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/motion_controller_node.h"

int main(int argc, char **argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "motion_controller_node");
    // ros::Rate rate(30);  // or your desired frequency

    try 
    {
        // Set tick frequency (in milliseconds)
        int TickPeriodMilliseconds = 100;  // Adjusted to 100ms for better real-time response

        // Create Behavior Tree Nodes
        BT::ManagerRunningCondition* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");
        BT::ManagerDoneCondition* is_manager_done = new BT::ManagerDoneCondition("IsManagerDone");
        BT::StartButtonCondition* is_start_button = new BT::StartButtonCondition("IsStartButton");
        BT::StandUp* stand_up = new BT::StandUp("StandUp");
        BT::BallDetectedCondition* ball_detected = new BT::BallDetectedCondition("BallDetected");
        BT::SearchBall* search_ball = new BT::SearchBall("SearchBall");
        BT::BallDirectionCondition* ball_direction = new BT::BallDirectionCondition("BallDirection");
        BT::TimerCondition* timer_condition = new BT::TimerCondition("TimerCondition", 5.0 /* seconds */);
        BT::SimpleWalk* walk_node = new BT::SimpleWalk("Walking");
        BT::TurnRight* turn_right = new BT::TurnRight("TurnRight");

        // Create Control Nodes
        BT::SequenceNodeWithMemory* root_node = new BT::SequenceNodeWithMemory("RootNode");
        BT::SequenceNodeWithMemory* init_sequence = new BT::SequenceNodeWithMemory("InitSequence");
        BT::SequenceNodeWithMemory* main_sequence = new BT::SequenceNodeWithMemory("MainLoop");
        BT::FallbackNode* fallback_node = new BT::FallbackNode("FallbackDecision");
        BT::SequenceNodeWithMemory* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
        BT::ParallelNode* parallel_walk_timer = new BT::ParallelNode("ParallelWalkTimer", 1, 1); // success_threshold=1, failure_threshold=1

        // Parallel node: walk and time concurrently
        parallel_walk_timer->addChild(walk_node);
        parallel_walk_timer->addChild(timer_condition);

        // Build the Behavior Tree
        init_sequence->AddChild(is_manager_running);
        init_sequence->AddChild(is_manager_done);
        init_sequence->AddChild(is_start_button);
        init_sequence->AddChild(stand_up);

        // Attach the fallback node to the main sequence
        main_sequence->AddChild(parallel_walk_timer);
        main_sequence->AddChild(stand_up);
        main_sequence->AddChild(turn_right);

        // Root node sequence
        root_node->AddChild(init_sequence);
        root_node->AddChild(main_sequence);

        // Execute the tree with the given tick period
        ros::spinOnce();
        Execute(root_node, TickPeriodMilliseconds);
    }
    catch (BT::BehaviorTreeException &Exception) 
    {
        std::cerr << "Behavior Tree Error: " << Exception.what() << std::endl;
    }

    return 0;
}
