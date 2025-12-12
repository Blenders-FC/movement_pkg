/* 
    Authors:
        Ricardo Berumen

*/
#include "rclcpp/rclcpp.hpp"
#include "movement_pkg/tree_builder.h"

int main() {
    rclppc::init(0, nullptr);
    BehaviorTreeFactory factory;
    // Build the tree here using factory
    auto tree = factory.createTreeFromFile("path_to_your_tree_file.xml");

    tree.tickRoot();
}

