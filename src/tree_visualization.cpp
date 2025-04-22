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

    BT::ControlNode* root_node = BT::TreeBuilder::BuildTree();
    BT::DotBt dotbt(root_node);

    dotbt.produceDot(root_node);  // Just generate the DOT string internally
    std::string dot_representation = dotbt.getDotFile();  // Fetch it

    std::ofstream file(dot_path);
    file << dot_representation;
    file.close();

    ROS_INFO_STREAM("DOT file written to: " << dot_path);
    
    return 0;
}
