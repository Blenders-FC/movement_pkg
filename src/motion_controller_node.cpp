/* 
Authors: Pedro Deniz
*/

#include <ros/ros.h>
#include "movement_pkg/MotionController.h"
#include <behavior_tree_core/SequenceNode.h>
#include <behavior_tree_core/FallbackNode.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_bt_main");
    ros::NodeHandle nh;

    // Root node
    BT::FallbackNode root("root");

    // Behavior nodes
    BallDetectedCondition ball_condition("BallDetected");
    MoveToBallAction move_action("MoveToBall");
    SearchBallAction search_action("SearchBall");
    HeadTrackingAction head_tracking("HeadTracking");
    FallRecoveryCondition fall_recovery("FallRecovery");
    KickAction kick_action("Kick");

    // Build behavior tree structure
    root.AddChild(&fall_recovery);
    root.AddChild(&ball_condition);
    root.AddChild(&head_tracking);
    root.AddChild(&move_action);
    root.AddChild(&kick_action);
    root.AddChild(&search_action);

    // Execute tree
    BT::Execute(&root, 100);

    return 0;
}
