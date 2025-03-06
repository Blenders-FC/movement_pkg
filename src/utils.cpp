/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/utils.h"

utils::utils() : nh(ros::this_node::getName()) 
{
    nh.param<int>("robot_id", robot_id, 0);
    ROS_INFO("Loaded utils: robot_id=%d", robot_id);
    
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");
    get_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis_" + std::to_string(robot_id) + "/get_present_ctrl_modules");
}

void utils::setModule(const std::string& module_name) {
    robotis_controller_msgs::SetModule set_module_srv;
    set_module_srv.request.module_name = module_name;
    action_pose_pub_ = nh.advertise<std_msgs::Int32>("/robotis_" + std::to_string(robot_id) + "/action/page_num", 0);

    ros::Duration(1.0).sleep();  //wait for module DO NOT REMOVE!!!!
    if (set_joint_module_client.call(set_module_srv) == false) {
        ROS_ERROR("Failed to set module");
        return;
    }
    return;
}

void utils::goAction(int page) {
    setModule("action_module");
    ROS_INFO("Action pose");
  
    action_msg_.data = page;
    action_pose_pub_.publish(action_msg_);
}

// Function to dynamically get the "data" folder path
std::string utils::getDataFilePath(const std::string& filename) 
{
    // Get the directory of the current source file (base_class.cpp)
    std::string source_path = __FILE__;  // Gets "/home/robotis/catkin_ws/src/<PACKAGE>/src/utils.cpp"

    // Move up one directory level to the package root
    fs::path base_path = fs::path(source_path).parent_path().parent_path();  
    // Now we have "/home/robotis/catkin_ws/src/<PACKAGE>"

    // Append the "data" folder and filename
    fs::path data_file = base_path / "data" / filename;

    return data_file.string();  // Convert to string and return
}

// Function to load positions from a file
std::vector<std::vector<float>> utils::loadPositions() 
{
    std::string filepath = getDataFilePath("Pararse.txt");
    std::ifstream myfile(filepath);
    
    if (!myfile.is_open()) {
        std::cerr << "Couldn't open file!" << std::endl;
        return {};  // Return empty vector if file fails to open
    }

    std::cout << "File was opened!!!" << std::endl;

    // Create a 2D vector initialized with zeros
    std::vector<std::vector<float>> positions(rows_, std::vector<float>(cols_, 0.0f));

    // Read data into the 2D vector
    for (int idx2 = 0; idx2 < rows_; idx2++) {
        for (int idy2 = 0; idy2 < cols_; idy2++) {
            if (!(myfile >> positions[idx2][idy2])) {
                std::cerr << "Error reading data at (" << idx2 << "," << idy2 << ")" << std::endl;
                return {};  // Return empty vector on error
            }
        }
    }

    myfile.close();
    return positions;
}
