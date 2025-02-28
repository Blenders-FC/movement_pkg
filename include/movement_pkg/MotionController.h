/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef MOTION_CONTROLLER_NODE_H
#define MOTION_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <behavior_tree_core/bt_factory.h>

// Include custom nodes
#include "nodes/ball_detected_condition.h"
#include "nodes/walk_action.h"
#include "nodes/kick_action.h"
#include "nodes/search_ball_action.h"
#include "nodes/robot_fallen_condition.h"

class MotionControllerNode {
public:
    MotionControllerNode();
    void run();

private:
    ros::NodeHandle nh_;
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
};

#endif // MOTION_CONTROLLER_NODE_H
