#ifndef CB_DATA_MANAGER_H
#define CB_DATA_MANAGER_H

#include <eigen3/Eigen/Eigen>

#include "movement_pkg/utils.h"
//#include "vision_pkg/referee.h"
#include "robotis_math/robotis_linear_algebra.h"


enum referee{
    STILL = 0,
    MIDFIELD,
    PLAY,
    GET_CLOSE,
    GET_FAR

};

class CBDataManager : public rclcpp::Node
{
public:
    CBDataManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // Constructor
    void init();
    // External functions
    geometry_msgs::msg::Point getBallPosition();
    double getRobotPitch();
    double getHeadPan();
    double getHeadTilt();
    // int getRefereeState();
    bool getStartButtonState();
    std::pair<std::string, std::string> getRobotStatus();
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<utils> utils_;
    int robot_id;

private:

    // Callbacks
    void ballCenterCallback(const geometry_msgs::msg::Point& msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState& msg);
    //void refereeCallback(const vision_pkg::referee& msg);
    void buttonHandlerCallback(const std_msgs::msg::String::ConstPtr& msg);
    void statusCallback(const robotis_controller_msgs::msg::StatusMsg::ConstPtr& msg);

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    //rclcpp::Subscription<vision_pkg::referee>::SharedPtr ref_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr button_sub_;
    rclcpp::Subscription<robotis_controller_msgs::msg::StatusMsg>::SharedPtr robot_status_sub_;

    // Variables
    geometry_msgs::msg::Point ball_position_;
    Eigen::Quaterniond imu_orientation_;
    Eigen::MatrixXd rpy_orientation_;
    double head_pan_;
    double head_tilt_;
    bool start_button_flag_;
    std::string module_name_;
    std::string status_msg_;
    //referee blackboard variable
    //TargetInfo m_refereeInfo;
    //imu
    double alpha = 0.4;
    double pitch;
    double rpy_orientation;
    const double FALL_FORWARD_LIMIT = 55;
    const double FALL_BACK_LIMIT = -55;
    double present_pitch_ = 0;
};

#endif  // CB_DATA_MANAGER_H
