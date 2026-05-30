/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/cb_data_manager.h"

// Private constructor (subscribes to topic)
CBDataManager::CBDataManager(const rclcpp::NodeOptions& options) : rclcpp::Node("cb_data_manager", options), imu_orientation_(1, 0, 0, 0)  // Default identity quaternion | This definition save computation
{   
    if (!this->has_parameter("robot_id")) {
        this->declare_parameter<int>("robot_id", 1);
    }
    robot_id = this->get_parameter("robot_id").as_int();

    const std::string prefix = "/robotis_" + std::to_string(robot_id);

    ball_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
	prefix + "/ball_center", 
	10, 
	std::bind(&CBDataManager::ballCenterCallback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      prefix + "/open_cr/imu",
      10,
      std::bind(&CBDataManager::imuCallback, this, std::placeholders::_1));

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      prefix + "/present_joint_states",
      10,
      std::bind(&CBDataManager::jointStatesCallback, this, std::placeholders::_1));

  /*ref_sub_ = this->create_subscription<vision_pkg::msg::Referee>(
      prefix + "/referee_data",
      10,
      std::bind(&CBDataManager::refereeCallback, this, std::placeholders::_1));
*/
  button_sub_ = this->create_subscription<std_msgs::msg::String>(
      prefix + "/open_cr/button",
      10,
      std::bind(&CBDataManager::buttonHandlerCallback, this, std::placeholders::_1));

  robot_status_sub_ = this->create_subscription<robotis_controller_msgs::msg::StatusMsg>(
      prefix + "/status",
      10,
      std::bind(&CBDataManager::statusCallback, this, std::placeholders::_1));
}
void CBDataManager::init() {
  utils_ = std::make_shared<utils>(this->shared_from_this());
}

// [============================== CALLBACKS ==============================]

// Updating latest ball position
void CBDataManager::ballCenterCallback(const geometry_msgs::msg::Point& msg)
{
    
    ball_position_.x = msg.x; // 320) - 1;
    ball_position_.y = msg.y; // 240) - 1;
}

// Updating IMU state
void CBDataManager::imuCallback(const sensor_msgs::msg::Imu::ConstPtr& msg)
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
        utils_->goAction(122);
        utils_->setModule("none");
    } 
    else if (present_pitch_ < FALL_BACK_LIMIT) 
    {
        utils_->goAction(1);
        utils_->setModule("none");
        //ros::Duration(1.0).sleep();
        rclcpp::sleep_for(std::chrono::seconds(1));
        utils_->goAction(82);
        utils_->setModule("none");
    }
}

// Updating head pan and tilt
void CBDataManager::jointStatesCallback(const sensor_msgs::msg::JointState& msg)
{ 
  head_pan_ = msg.position[0];
  head_tilt_ = msg.position[1];
}

// Updating referee state
/*
void CBDataManager::refereeCallback(const vision_pkg::referee& msg)
{
    
    0 = "quieto"
    1 = "acomodate"
    2 = "playing"
    3 = "acercate"
    4 = "alejate"
    
//check if previous state was the same
    if(blackboard.getTarget("m_refereeStatus")->refereeStatus == msg.robotPlayStateInt){
        m_refereeInfo.refereeStatus = msg.robotPlayStateInt; 
        blackboard.setTarget("m_refereeStatus",m_refereeInfo);
        // ROS_COLORED_LOG("refereeState stable: %d", CYAN, true, msg.robotPlayStateInt);

        return; //if so return, there is nothing to change, only update blackboard
    }
    m_refereeInfo.refereeStatus = msg.robotPlayStateInt; 
    blackboard.setTarget("m_refereeStatus",m_refereeInfo); //update blacboard


    //if statements to change behavior if referee changed its state


switch (msg.robotPlayStateInt)
{
    case referee::STILL:
        // code 
        ROS_COLORED_LOG("refereeState changed to STILL: %d", CYAN, true, msg.robotPlayStateInt);
        break;
    case referee::MIDFIELD:
        ROS_COLORED_LOG("refereeState changed to MIDFIELD kickoff: %d", CYAN, true, msg.robotPlayStateInt);
        break;
    case referee::PLAY:
        ROS_COLORED_LOG("refereeState changed to PlAY: %d", CYAN, true, msg.robotPlayStateInt);
        break;
    case referee::GET_CLOSE:
        ROS_COLORED_LOG("refereeState changed to GET_CLOSE to ball: %d", CYAN, true, msg.robotPlayStateInt);
        break;
    case referee::GET_FAR:
        ROS_COLORED_LOG("refereeState changed to GET_FAR from ball: %d", CYAN, true, msg.robotPlayStateInt);
        break;
    default:
        break;
    }
}
*/
// Updating start button state
void CBDataManager::buttonHandlerCallback(const std_msgs::msg::String::ConstPtr& msg)
{
    if (msg->data == "mode") {
      start_button_flag_ = true;
    } else if (msg->data == "start"){
      start_button_flag_ = false;
    }
}

// Updating general robot status
void CBDataManager::statusCallback(const robotis_controller_msgs::msg::StatusMsg::ConstPtr& msg)
{
    // Save data into global variables
    module_name_ = msg->module_name;
    status_msg_ = msg->status_msg;
}


// [========================= EXTERNAL FUNCTIONS ==========================]

geometry_msgs::msg::Point CBDataManager::getBallPosition()
{
    return ball_position_;
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
    RCLCPP_INFO(this->get_logger(), "Called Start Button!");
    return start_button_flag_;
}

std::pair<std::string, std::string> CBDataManager::getRobotStatus()
{
    RCLCPP_INFO(this->get_logger(), "Called Robot Status!");
    RCLCPP_INFO(this->get_logger(), "Module: %s, Status: %s", module_name_.c_str(), status_msg_.c_str());
    return std::make_pair(module_name_, status_msg_);
}
