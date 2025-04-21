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
    
    // Set tick frequency (in milliseconds)
    int TickPeriodMilliseconds = 100;  // Adjusted to 100ms for better real-time response
    
    // ros::Rate rate(30);  // or your desired frequency
    ros::spinOnce();

    try 
    {

        // Execute the tree with the given tick period
        auto* root_node = BT::BehaviorTreeBuilder::BuildTree();
        Execute(root_node, TickPeriodMilliseconds);
    }
    catch (BT::BehaviorTreeException &Exception) 
    {
        std::cerr << "Behavior Tree Error: " << Exception.what() << std::endl;
    }

    return 0;
}
