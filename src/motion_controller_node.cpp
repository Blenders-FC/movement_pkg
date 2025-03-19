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

        // Create Control Nodes
        BT::SequenceNodeWithMemory* root_sequence = new BT::SequenceNodeWithMemory("RootSequence");
        BT::FallbackNode* fallback_node = new BT::FallbackNode("FallbackDecision");

        // Build the Behavior Tree
        root_sequence->AddChild(is_manager_running);
        root_sequence->AddChild(is_manager_done);
        root_sequence->AddChild(is_start_button);
        root_sequence->AddChild(stand_up);

        // Decision Making Structure
        BT::SequenceNodeWithMemory* ball_found_sequence = new BT::SequenceNodeWithMemory("BallFoundSequence");
        ball_found_sequence->AddChild(ball_detected);
        ball_found_sequence->AddChild(ball_direction);

        // Add sequences to fallback
        fallback_node->AddChild(ball_found_sequence);
        fallback_node->AddChild(search_ball);

        // Attach the fallback node to the root sequence
        root_sequence->AddChild(fallback_node);

        // Execute the tree with the given tick period
        Execute(root_sequence, TickPeriodMilliseconds);
    }
    catch (BT::BehaviorTreeException &Exception) 
    {
        std::cerr << "Behavior Tree Error: " << Exception.what() << std::endl;
    }

    return 0;
}
