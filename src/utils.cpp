/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/utils.h"
#include <chrono>
using namespace std::chrono_literals;

// Static map definition
std::unordered_map<std::string, bool> utils::already_logged_tags_;

utils::utils() :
    blackboard()
{
    node_ = rclcpp::Node::make_shared("utils");

    // ROS2 parameter API
    node_->declare_parameter<int>("robot_id", 0);
    robot_id = node_->get_parameter("robot_id").as_int();

    RCLCPP_INFO(node_->get_logger(), "Loaded utils (ROS2): robot_id=%d", robot_id);

    // Create service clients
    set_joint_module_client_ =
        node_->create_client<robotis_controller_msgs::srv::SetModule>(
            "/robotis_" + std::to_string(robot_id) + "/set_present_ctrl_modules");

    get_joint_module_client_ =
        node_->create_client<robotis_controller_msgs::srv::GetJointModule>(
            "/robotis_" + std::to_string(robot_id) + "/get_present_joint_ctrl_modules");

    // Publisher
    action_pose_pub_ =
        node_->create_publisher<std_msgs::msg::Int32>(
            "/robotis_" + std::to_string(robot_id) + "/action/page_num", 10);
}

void utils::setModule(const std::string& module_name)
{
    auto req = std::make_shared<robotis_controller_msgs::srv::SetModule::Request>();
    req->module_name = module_name;
    last_module = module_name;

    rclcpp::sleep_for(1s);   // KEEP THIS DELAY

    if (!set_joint_module_client_->wait_for_service(2s))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service set_present_ctrl_modules not available!");
        return;
    }

    auto future = set_joint_module_client_->async_send_request(req);

    // Optional: wait synchronously
    if (rclcpp::spin_until_future_complete(node_, future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call SetModule");
        return;
    }
}

std::string utils::getModule(const std::string& joint_name)
{
    auto req = std::make_shared<robotis_controller_msgs::srv::GetJointModule::Request>();
    req->joint_name.push_back(joint_name);

    if (!get_joint_module_client_->wait_for_service(2s))
    {
        RCLCPP_ERROR(node_->get_logger(), "Service get_present_joint_ctrl_modules not available!");
        return "";
    }

    auto future = get_joint_module_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed calling get_present_joint_ctrl_modules for joint '%s'",
                     joint_name.c_str());
        return "";
    }

    auto result = future.get();
    if (result->module_name.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "Empty joint module response!");
        return "";
    }

    return result->module_name.front();
}

void utils::goAction(int page)
{
    setModule("action_module");
    RCLCPP_INFO(node_->get_logger(), "Action pose");

    std_msgs::msg::Int32 msg;
    msg.data = page;
    action_pose_pub_->publish(msg);
}

std::string utils::getDataFilePath(const std::string& filename)
{
    std::string source_path = __FILE__;

    std::size_t pos = source_path.rfind("/src/");
    if (pos == std::string::npos)
    {
        std::cerr << "Error: Could not determine package root from __FILE__." << std::endl;
        return "";
    }

    std::string base_path = source_path.substr(0, pos);
    std::string data_file = base_path + "/config/" + filename;

    std::cerr << data_file << std::endl;
    return data_file;
}

std::vector<std::vector<float>> utils::loadPositions()
{
    std::string filepath = getDataFilePath("StandUpPositions.txt");
    std::ifstream myfile(filepath);

    if (!myfile.is_open())
    {
        std::cerr << "Couldn't open StandUpPositions.txt file!" << std::endl;
        return {};
    }

    RCLCPP_INFO(node_->get_logger(), "StandUpPositions.txt file opened.");

    std::vector<std::vector<float>> positions(rows_, std::vector<float>(cols_, 0.0f));

    for (int r = 0; r < rows_; r++)
    {
        for (int c = 0; c < cols_; c++)
        {
            if (!(myfile >> positions[r][c]))
            {
                std::cerr << "Error reading (" << r << "," << c << ")" << std::endl;
                return {};
            }
        }
    }

    myfile.close();
    return positions;
}

const char* utils::resolveColor(const std::string& color, bool bold)
{
    auto it = color_map.find(color);
    if (it != color_map.end())
        return bold ? it->second.second : it->second.first;
    return DEFAULT_TEXT;
}

void utils::ROS_TAGGED_ONCE_LOG(const std::string& msg,
                                const std::string& color,
                                bool bold,
                                const std::string& tag)
{
    std::string resolved_tag = tag.empty() ? "__default__" : tag;

    if (!already_logged_tags_[resolved_tag] && DEBUG_PRINT)
    {
        const char* color_str = resolveColor(color, bold);
        RCLCPP_INFO(node_->get_logger(), "%s%s%s", color_str, msg.c_str(), DEFAULT_TEXT);
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

void utils::resetLoggedTags()
{
    already_logged_tags_.clear();
}
