/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/motion_controller_node.h"

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "motion_controller_node");

    try {
        // Set tick frequency (in milliseconds)
        int TickPeriodMilliseconds = 100;  // Adjusted to 100ms for better real-time response

        // Create Behavior Tree Nodes
        BT::BallDetectedCondition* ball_detected = new BT::BallDetectedCondition("BallDetected");
        BT::WalkToTarget* walk_to_ball = new BT::WalkToTarget("WalkToTarget");
        BT::ManagerRunningCondition* is_manager_running = new BT::ManagerRunningCondition("IsManagerRunning");

        // Create Control Nodes
        BT::SequenceNodeWithMemory* root_sequence = new BT::SequenceNodeWithMemory("RootSequence");

        // Build the Behavior Tree
        root_sequence->AddChild(is_manager_running);
        root_sequence->AddChild(ball_detected);
        root_sequence->AddChild(walk_to_ball);

        // Execute the tree with the given tick period
        Execute(root_sequence, TickPeriodMilliseconds);
    }
    catch (BT::BehaviorTreeException &Exception) {
        std::cerr << "Behavior Tree Error: " << Exception.what() << std::endl;
    }

    return 0;
}
