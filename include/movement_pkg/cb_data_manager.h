#ifndef CB_DATA_MANAGER_H
#define CB_DATA_MANAGER_H

#include <eigen3/Eigen/Eigen>

#include "movement_pkg/utils.h"
// #include "soccer_pkg/referee.h"
#include "robotis_math/robotis_linear_algebra.h"


enum referee{
    STILL = 0,
    MIDFIELD,
    PLAY,
    GET_CLOSE,
    GET_FAR

};

class CBDataManager : public virtual utils
{
public:
    CBDataManager();  // Constructor

    // External functions
    geometry_msgs::Point getBallPosition();
    double getRobotPitch();
    double getHeadPan();
    double getHeadTilt();
    // int getRefereeState();
    bool getStartButtonState();
    std::pair<std::string, std::string> getRobotStatus();

private:

    // Callbacks
    void ballCenterCallback(const geometry_msgs::Point& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState& msg);
    // void refereeCallback(const soccer_pkg::referee& msg);
    void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
    void statusCallback(const robotis_controller_msgs::StatusMsg::ConstPtr& msg);

    // Subscribers
    ros::Subscriber ball_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber read_joint_sub_;
    // ros::Subscriber ref_sub_;
    ros::Subscriber button_sub_;
    ros::Subscriber robot_status_sub_;
    
    // Variables
    geometry_msgs::Point ball_position_;
    Eigen::Quaterniond imu_orientation_;
    Eigen::MatrixXd rpy_orientation_;
    double head_pan_;
    double head_tilt_;
    bool start_button_flag_;
    std::string module_name_;
    std::string status_msg_;
    //referee blackboard variable
    TargetInfo m_refereeInfo;
    //imu
    double alpha = 0.4;
    double pitch;
    double rpy_orientation;
    const double FALL_FORWARD_LIMIT = 55;
    const double FALL_BACK_LIMIT = -55;
    double present_pitch_ = 0;
};

#endif  // CB_DATA_MANAGER_H
