/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/cb_data_manager.h"

// Private constructor (subscribes to topic)
CBDataManager::CBDataManager() : utils(), imu_orientation_(1, 0, 0, 0)  // Default identity quaternion | This definition save computation
{
    // Subscribers
    ball_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/ball_center", 10, &CBDataManager::ballCenterCallback, this);
    goals_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/goals_centers", 10, &CBDataManager::goalCenterCallback, this);
    imu_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/imu", 10, &CBDataManager::imuCallback, this);
    read_joint_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/present_joint_states", 10, &CBDataManager::jointStatesCallback, this);
    // ref_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/r_data", 10, &CBDataManager::refereeCallback, this);
    button_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/button", 10, &CBDataManager::buttonHandlerCallback, this);
    robot_status_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/status", 10, &CBDataManager::statusCallback, this);
    robot_init_pose_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/robot_pose/init_pose", 10, &CBDataManager::initPoseCallback, this);
}

// [============================== CALLBACKS ==============================]

// Updating latest ball position
void CBDataManager::ballCenterCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ball_position_.x = msg->x; // 320) - 1;
    ball_position_.y = msg->y; // 240) - 1;
}

// Updating latest goal position
void CBDataManager::goalCenterCallback(const blenders_msgs::PointArray::ConstPtr& msg)
{
    ROS_INFO("Received %lu goal posts", msg->points.size());
    goals_positions_ = msg->points;
}

// Updating IMU state
void CBDataManager::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
    rpy_orientation *= (180 / 3.141516);
    
    pitch = rpy_orientation.coeff(1, 0);

    if (present_pitch_ == 0) 
        present_pitch_ = pitch;
    else
        present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;
    // std::cout << present_pitch_ << std::endl;
    if (present_pitch_ > FALL_FORWARD_LIMIT) 
    {
        goAction(122);
        setModule("none");
    } 
    else if (present_pitch_ < FALL_BACK_LIMIT) 
    {
        goAction(1);
        setModule("none");
        ros::Duration(1.0).sleep();
        goAction(82);
        setModule("none");
    }
}

// Updating head pan and tilt
void CBDataManager::jointStatesCallback(const sensor_msgs::JointState& msg)
{ 
  head_pan_ = msg.position[0];
  head_tilt_ = msg.position[1];
}

// Updating referee state
// void CBDataManager::refereeCallback(const soccer_pkg::referee& msg)
// {
//     /*
//     0 = "quieto"
//     1 = "acomodate"
//     2 = "playing"
//     3 = "acercate"
//     4 = "alejate"
//     */
//     referee_state_ = msg;
// }

// Updating start button state
void CBDataManager::buttonHandlerCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "mode") 
    {
      start_button_flag_ = true;
    } 
    else if (msg->data == "start")
    {
      start_button_flag_ = false;
    }
}

// Updating general robot status
void CBDataManager::statusCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg)
{
    // Save data into global variables
    module_name_ = msg->module_name;
    status_msg_ = msg->status_msg;
}

// Updating init robot pose
void CBDataManager::initPoseCallback(const blenders_msgs::RobotPose::ConstPtr& msg)
{
    // Save data into global variable
    init_robot_pose_ = msg->pose;
    valid_init_robot_pose_ = msg->valid;
}


// [========================= EXTERNAL FUNCTIONS ==========================]

geometry_msgs::Point CBDataManager::getBallPosition()
{
    return ball_position_;
}

std::vector<geometry_msgs::Point> CBDataManager::getGoalsPositions()
{
    return goals_positions_;
}

double CBDataManager::getRobotPitch()
{
    rpy_orientation_ = robotis_framework::convertQuaternionToRPY(imu_orientation_);
    rpy_orientation_ *= (57.29578);  // 180 / 3.141516 -> rad to deg
    
    return rpy_orientation_.coeff(1, 0);  // IMU Pitch
}

double CBDataManager::getHeadPan()
{
    return head_pan_;
}

double CBDataManager::getHeadTilt()
{
    return head_tilt_;
}

// int CBDataManager::getRefereeState()
// {
//     return referee_state_;
// }

bool CBDataManager::getStartButtonState()
{
    return start_button_flag_;
}

std::pair<std::string, std::string> CBDataManager::getRobotStatus()
{
    return std::make_pair(module_name_, status_msg_);
}

geometry_msgs::Pose CBDataManager::getInitRobotPose()
{
    return init_robot_pose_;
}

bool CBDataManager::isInitPoseValid()
{
    return valid_init_robot_pose_;
}
