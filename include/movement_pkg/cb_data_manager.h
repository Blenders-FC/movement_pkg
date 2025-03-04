#ifndef CB_DATA_MANAGER_H
#define CB_DATA_MANAGER_H

#include <eigen3/Eigen/Eigen>

#include "movement_pkg/utils.h"
#include "soccer_pkg/referee.h"
#include "robotis_math/robotis_linear_algebra.h"

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

private:

    // Callbacks
    void ballCenterCallback(const geometry_msgs::Point& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void jointStatesCallback(const sensor_msgs::JointState& msg);
    // void refereeCallback(const soccer_pkg::referee& msg);
    void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);

    // Subscribers
    ros::Subscriber ball_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber read_joint_sub_;
    // ros::Subscriber ref_sub_;
    ros::Subscriber button_sub_;
    
    // Variables
    geometry_msgs::Point ball_position_;
    Eigen::Quaterniond imu_orientation_;
    Eigen::MatrixXd rpy_orientation_;
    double head_pan_;
    double head_tilt_;
    // int referee_state_;
    bool start_button_flag_;
};

#endif  // CB_DATA_MANAGER_H
