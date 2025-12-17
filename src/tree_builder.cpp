/* 
    Authors:
        Ricardo Berumen

*/
//#include "movement_pkg/tree_builder.h"

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
//#include <behaviortree_cpp/loggers/bt_zmq_publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("bt_executor");

    BT::BehaviorTreeFactory factory;

    // --------------------------------------------------
    // Load BT plugin (your ManagerRunningCondition)
    // --------------------------------------------------
    const auto pkg_share =
        ament_index_cpp::get_package_share_directory("movement_pkg");

    const auto plugin_path =
        std::filesystem::path(pkg_share).parent_path().parent_path()
        / "lib/libmanager_bt_plugin.so";

    factory.registerFromPlugin(plugin_path.string());

    // --------------------------------------------------
    // Load BT XML
    // --------------------------------------------------
    const auto tree_xml =
        pkg_share + "/behavior_trees/init_check.xml";

    auto tree = factory.createTreeFromFile(tree_xml);

    // Optional: console logger
    BT::StdCoutLogger logger(tree);

    // Optional: Groot2 live visualization
    //BT::PublisherZMQ zmq_publisher(tree);

    rclcpp::Rate rate(10);

    RCLCPP_INFO(node->get_logger(), "Behavior Tree executor started");

    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
