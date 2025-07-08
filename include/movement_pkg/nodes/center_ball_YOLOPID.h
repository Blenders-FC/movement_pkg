/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CENTER_BALL_YOLOPID_ACTION_H
#define CENTER_BALL_YOLOPID_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>

struct PIDController
{
    double Kp;
    double Ki;
    double Kd;

    double integral;
    double prev_error;
    double integral_limit;

    PIDController(double p, double i, double d, double i_limit = 0.2)
        : Kp(p), Ki(i), Kd(d), integral(0), prev_error(0), integral_limit(i_limit) {}
};

namespace BT
{
class CenterBallYOLOPID : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit CenterBallYOLOPID(std::string name);
        ~CenterBallYOLOPID();

        // The method executed by the thread
        void WaitForTick();

        // Used to interrupt execution of the node
        void Halt();

    private:
        // Auxiliar methods
        void writeHeadJoint(double ang_value, bool is_pan);
        double clamp(double value, double min_value, double max_value);
        void resetPID();

        // ROS
        ros::Publisher write_joint_pub_;

        // PID controllers for pan and tilt
        PIDController pid_pan_{2.0, 0.0, 0.1};   // Example initial gains
        PIDController pid_tilt_{2.0, 0.0, 0.1};

        // Constants for pixel-to-deg conversion
        static constexpr double X_PIXEL_TO_DEG = 0.21875;
        static constexpr double Y_PIXEL_TO_DEG = 0.29166;

        // Head joint angle limits (rad)
        static constexpr double PAN_MAX_RAD = 1.2217;   // ~70 deg
        static constexpr double PAN_MIN_RAD = -1.2217;
        static constexpr double TILT_MAX_RAD = 0.34906; // ~20 deg
        static constexpr double TILT_MIN_RAD = -1.2217;

        // Variables
        geometry_msgs::Point ball_center_position_;
        double dt = 1.0 / 30.0; // 30 Hz

        double head_pan_angle_;
        double head_tilt_angle_;
        double angle_mov_x_;
        double angle_mov_y_;
        double xerror_;
        double yerror_;

        sensor_msgs::JointState write_msg_;
};
}  // namespace BT

#endif  // CENTER_BALL_YOLOPID_ACTION_H
