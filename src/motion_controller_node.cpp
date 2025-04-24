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
    std::cout << DEFAULT_TEXT       << "DEFAULT_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_DEFAULT_TEXT  << "BOLD_DEFAULT_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << RED_TEXT           << "RED_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_RED_TEXT      << "BOLD_RED_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << GREEN_TEXT         << "GREEN_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_GREEN_TEXT    << "BOLD_GREEN_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << YELLOW_TEXT        << "YELLOW_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_YELLOW_TEXT   << "BOLD_YELLOW_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << BLUE_TEXT          << "BLUE_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_BLUE_TEXT     << "BOLD_BLUE_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << MAGENTA_TEXT       << "MAGENTA_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_MAGENTA_TEXT  << "BOLD_MAGENTA_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << CYAN_TEXT          << "CYAN_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_CYAN_TEXT     << "BOLD_CYAN_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << ORANGE_TEXT        << "ORANGE_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_ORANGE_TEXT   << "BOLD_ORANGE_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << VIOLET_TEXT        << "VIOLET_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_VIOLET_TEXT   << "BOLD_VIOLET_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << PINK_TEXT          << "PINK_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_PINK_TEXT     << "BOLD_PINK_TEXT" << DEFAULT_TEXT << std::endl;

    std::cout << TEAL_TEXT          << "TEAL_TEXT" << DEFAULT_TEXT << std::endl;
    std::cout << BOLD_TEAL_TEXT     << "BOLD_TEAL_TEXT" << DEFAULT_TEXT << std::endl;

    // Backgrounds
    std::cout << RED_BG_TEXT        << "RED_BG_TEXT       " << DEFAULT_TEXT << std::endl;
    std::cout << GREEN_BG_TEXT      << "GREEN_BG_TEXT     " << DEFAULT_TEXT << std::endl;
    std::cout << YELLOW_BG_TEXT     << "YELLOW_BG_TEXT    " << DEFAULT_TEXT << std::endl;
    std::cout << BLUE_BG_TEXT       << "BLUE_BG_TEXT      " << DEFAULT_TEXT << std::endl;
    std::cout << MAGENTA_BG_TEXT    << "MAGENTA_BG_TEXT   " << DEFAULT_TEXT << std::endl;
    std::cout << CYAN_BG_TEXT       << "CYAN_BG_TEXT      " << DEFAULT_TEXT << std::endl;
    std::cout << ORANGE_BG_TEXT     << "ORANGE_BG_TEXT    " << DEFAULT_TEXT << std::endl;
    std::cout << VIOLET_BG_TEXT     << "VIOLET_BG_TEXT    " << DEFAULT_TEXT << std::endl;
    std::cout << TEAL_BG_TEXT       << "TEAL_BG_TEXT      " << DEFAULT_TEXT << std::endl;
    std::cout << BLACK_BG_TEXT      << "BLACK_BG_TEXT     " << DEFAULT_TEXT << std::endl;
    std::cout << GRAY_BG_TEXT       << "GRAY_BG_TEXT      " << DEFAULT_TEXT << std::endl;
    std::cout << BROWN_BG_TEXT      << "BROWN_BG_TEXT     " << DEFAULT_TEXT << std::endl;

    try 
    {
        // Execute the tree with the given tick period
        auto* root_node = BT::TreeBuilder::BuildTree();
        Execute(root_node, TickPeriodMilliseconds);
    }
    catch (BT::BehaviorTreeException &Exception) 
    {
        std::cerr << "Behavior Tree Error: " << Exception.what() << std::endl;
    }

    return 0;
}
