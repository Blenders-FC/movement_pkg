/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <fstream>
#include "movement_pkg/tree_builder.h"

int main()
{   
    // execution command:
    //dot -Tpng bt_tree.dot -o bt_tree.png

    std::ofstream file("bt_tree.dot");
    file << "digraph BehaviorTree {\n";
    int node_id = 0;

    TreeNode* root_node = BT::TreeBuilder::BuildTree();
    root_node->GetDotRepresentation(node_id, file);

    file << "}\n";
    file.close();

    std::cout << "DOT file written to bt_tree.dot\n";
}
