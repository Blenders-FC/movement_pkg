/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/utils.h"


utils::utils() : nh(ros::this_node::getName()), blackboard()
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
        ROS_ERROR_LOG("Failed to set module", false);
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
    // Get the directory of the current source file (utils.cpp)
    std::string source_path = __FILE__;  // Gets "/home/robotis/catkin_ws/src/<PACKAGE>/src/utils.cpp"

    // Find the last occurrence of "/src/" in the path to get the package root
    std::size_t pos = source_path.find("/src/");
    if (pos == std::string::npos) {
        std::cerr << "Error: Could not determine package root from __FILE__." << std::endl;
        return "";  // Return an empty string in case of an error
    }

    // Extract the package root path
    std::string base_path = source_path.substr(0, pos);  // "/home/robotis/catkin_ws/src/<PACKAGE>"

    // Construct the final file path
    std::string data_file = base_path + "/data/" + filename;

    return data_file;
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

const char* utils::resolveColor(const std::string& color, bool bold) {
    auto it = color_map.find(color);
    if (it != color_map.end()) {
        return bold ? it->second.second : it->second.first;
    }
    return DEFAULT_TEXT;
}

/*
 * Logs a message only once, with an optional tag, color, and boldness.
 * @param msg The message to log.
 * @param color The color name (e.g., CYAN, GREEN).
 * @param bold Whether the message should be bold.
 * @param tag Optional tag. If empty, logs only once globally.
*/
void utils::ROS_TAGGED_ONCE_LOG(const std::string& msg,
    const std::string& color,
    bool bold,
    const std::string& tag) 
{
    std::string resolved_tag = tag.empty() ? "__default__" : tag;
    
    if (!already_logged_tags_[resolved_tag] && DEBUG_PRINT)
    {
        const char* resolved_color = resolveColor(color, bold);
        ROS_COLORED_ONCE_LOG("%s", resolved_color, bold, msg.c_str());
        already_logged_tags_[resolved_tag] = true;
    }
}

std::unordered_map<std::string, std::pair<const char*, const char*>> utils::color_map = {
    {"RED",    {RED_TEXT,    BOLD_RED_TEXT}},
    {"GREEN",  {GREEN_TEXT,  BOLD_GREEN_TEXT}},
    {"YELLOW", {YELLOW_TEXT, BOLD_YELLOW_TEXT}},
    {"PINK",   {PINK_TEXT,   BOLD_PINK_TEXT}},
    {"CYAN",   {CYAN_TEXT,   BOLD_CYAN_TEXT}},
};
