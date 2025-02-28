/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/utils.h"

void utils::setModule(const std::string& module_name) {
    robotis_controller_msgs::SetModule set_module_srv;
    set_module_srv.request.module_name = module_name;

    if (set_joint_module_client.call(set_module_srv) == false) {
        ROS_ERROR("Failed to set module");
        return;
    }
    return;
}

void utils::goAction(int page) {
    setModule("action_module");
    ROS_INFO("Action pose");
  
    std_msgs::Int32 action_msg;
    action_msg.data = page;
    action_pose_pub.publish(action_msg);
}
