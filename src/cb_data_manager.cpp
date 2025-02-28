/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/cb_data_manager.h"

// Private constructor (subscribes to topic)
CBDataManager::CBDataManager() : imu_orientation_(1, 0, 0, 0){  // Default identity quaternion | This definition save computation
    // Subscribers
    ball_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/ball_center", 10, CBDataManager::ballCenterCallback);
    imu_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/imu", 10, CBDataManager::imuCallback);
    read_joint_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/present_joint_states", 10, CBDataManager::jointStatesCallback);
    ref_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/r_data", 10, CBDataManager::refereeCallback);
    button_sub_ = nh.subscribe("/robotis_" + std::to_string(robot_id) + "/open_cr/button", 10, CBDataManager::buttonhandlerCallback);
}

// Singleton instance
CBDataManager& CBDataManager::getInstance() {
    static CBDataManager instance(nh);
    return instance;
}

// [============================== CALLBACKS ==============================]

// Updating latest ball position
void CBDataManager::ballCenterCallback(const geometry_msgs::Point::ConstPtr &msg) {
    ball_position_.x = msg.x; // 320) - 1;
    ball_position_.y = msg.y; // 240) - 1;
}

// Updating IMU state
void CBDataManager::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_orientation_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
}

// Updating head pan and tilt
void CBDataManager::jointStatesCallback(const sensor_msgs::JointState& msg) { 
  head_pan_ = msg.position[0];
  head_tilt_ = msg.position[1];
}

// Updating referee state
void callbackReferee(const soccer_pkg::referee& msg) {
    /*
    0 = "quieto"
    1 = "acomodate"
    2 = "playing"
    3 = "acercate"
    4 = "alejate"
    */
    referee_state_ = msg.I;
}

// Updating start button state
void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "mode") {
      start_button_flag_ = true;
    } else if (msg->data == "start"){
      start_button_flag_ = false;
    }
}


// [========================= EXTERNAL FUNCTIONS ==========================]

geometry_msgs::Point CBDataManager::getBallPosition() {
    return ball_position_;
}

double CBDataManager::getRobotPitch(){
    rpy_orientation_ = robotis_framework::convertQuaternionToRPY(imu_orientation_);
    rpy_orientation_ *= (57.29578);  // 180 / 3.141516 -> rad to deg
    
    return rpy_orientation_.coeff(1, 0);  // IMU Pitch
}

double CBDataManager::getHeadPan(){
    return head_pan_;
}

double CBDataManager::getHeadTilt(){
    return head_tilt_;
}

int CBDataManager::getRefereeState(){
    return referee_state_;
}

bool getStartButtonState(){
    return start_button_flag_;
}
