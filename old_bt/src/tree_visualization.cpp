/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "dot_bt.h"
#include "movement_pkg/tree_builder.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tree_visualization");
    ros::NodeHandle nh("~");

    std::string dot_path;
    nh.param<std::string>("dot_output_path", dot_path, "bt_tree.dot");

    BT::ControlNode* tree = nullptr;

    try 
    {
        tree = BT::TreeBuilder::BuildTree();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to build behavior tree: " << e.what());
        return 1;
    }

    if (!tree) {
        ROS_ERROR("Tree is null! Exiting.");
        return 1;
    }    

    BT::DotBt dotbt(tree);

    dotbt.produceDot(tree);  // Just generate the DOT string internally
    std::string dot_representation = dotbt.getDotFile();  // Fetch it

    std::ofstream file(dot_path);
    file << dot_representation;
    file.close();

    ROS_INFO_STREAM("DOT file written to: " << dot_path);
    
    return 0;
}
