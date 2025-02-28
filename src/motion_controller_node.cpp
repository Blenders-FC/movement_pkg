/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/MotionController.h"
#include "movement_pkg/nodes/ball_detected_condition.h"

MotionControllerNode::MotionControllerNode() {
    ROS_INFO("Initializing Movement Controller...");

    // Register custom nodes
    factory_.registerNodeType<BallDetectedCondition>("BallDetected");
    factory_.registerNodeType<WalkAction>("Walk");
    factory_.registerNodeType<KickAction>("Kick");
    factory_.registerNodeType<SearchBallAction>("SearchBall");
    factory_.registerNodeType<RobotFallenCondition>("RobotFallen");

    // Load tree from XML
    std::string xml_path = ros::package::getPath("movement_pkg") + "/config/soccer_motion_controller.xml";
    tree_ = factory_.createTreeFromFile(xml_path);
}

void MotionControllerNode::run() {
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        tree_.tickRoot();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_controller_node");
    ros::NodeHandle nh(ros::this_node::getName());  // NodeHandle definition

    BallDetectedCondition ball_detected_condition("ball_detected_condition", nh)

    MotionControllerNode bt_node;  // BehaviorTree Node
    bt_node.run();
    return 0;
}
