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
  
    action_msg.data = page;
    action_pose_pub_.publish(action_msg);
}
