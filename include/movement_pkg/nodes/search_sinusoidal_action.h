/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef SEARCH_BALL_SINUSOIDAL_ACTION_H
#define SEARCH_BALL_SINUSOIDAL_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <cmath>

namespace BT
{
class SearchBallSinusoidal : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit SearchBallSinusoidal(std::string name);
        ~SearchBallSinusoidal();

        // Executed by the thread
        void WaitForTick();

        // Interrupts execution
        void Halt();

    private:
        // Helper to send joint commands
        void writeHeadJoint(double ang_valueX, double ang_valueY);

        // ROS publisher
        ros::Publisher write_joint_pub_;

        // State variables
        bool head_direction_ = true;
        double angle_mov_x_ = 0.0;
        double angle_mov_y_ = 0.0;
        double head_pan_angle_ = 0.0;
        double head_tilt_angle_ = 0.0;
        int turn_cnt_ = 0;

        // Sinusoidal search variables
        double t_ = 0.0;           // time or step counter
        double x_target_ = 0.0;    // target pan angle (deg)
        double x_target_past_ = 0.0;    // target pan angle (deg)
        double dx = 0.0;
        double y_target_ = 0.0;    // target tilt angle (deg)
        bool rightLeft=0;
        bool headDown=0;

        // Constants for OP3 head limits (rad)
        static constexpr double PAN_MAX_RAD = 1.22173;   // 70 deg
        static constexpr double PAN_MIN_RAD = -1.22173;  // -70 deg
        static constexpr double TILT_MAX_RAD = 0.34906;  // 20 deg
        static constexpr double TILT_MIN_RAD = -1.22173; // -70 deg

        // JointState messages for publishing (use local msg in function instead)
        sensor_msgs::JointState write_msg;
};
}  // namespace BT

#endif  // SEARCH_BALL_SINUSOIDAL_ACTION_H
