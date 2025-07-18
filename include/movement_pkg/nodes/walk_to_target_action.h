/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_TARGET_ACTION_H
#define WALKING_TO_TARGET_ACTION_H

#include "movement_pkg/walking_controller.h"
#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class WalkToTarget : public ActionNode, public WalkingController, public CBDataManager
{
    public:
        // Constructor
        explicit WalkToTarget(std::string name);
        ~WalkToTarget();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:
        //  Auxiliar methods
        void walkTowardsTarget(double head_pan_angle, double head_tilt_angle);
        void writeHeadJoint(double ang_value);
        double calculateDistance(double head_tilt);
        double calculateTilt(double remaining_distance);

        // ROS variable
        ros::Publisher write_joint_pub_;
        ros::Time prev_time_walk_ = ros::Time::now();
        sensor_msgs::JointState write_msg_;


        // Variables
        double walked_distance = 0.0;
        double accum_rotation = 0.0;
        double prev_delta_angle = 0.0;
        double head_pan_angle_;
        double head_tilt_angle_;
        double fb_move;
        double rl_angle;
        double distance_to_walk;
        const double distance_to_kick_ = 0.15; //0.0;  // 0.30;  // 0.22
        const double CAMERA_HEIGHT_ = 0.46;
        const double hip_pitch_offset_ = 0.12217305; //7°
        bool walkingSucced = false;
        bool walkLimitReach = false;
        const double walk_thresh  = 2.0;
        std_msgs::String walk_command;
        //ros::Time prev_time_walk_ = ros::Time::now();
};
}  // namespace BT

#endif  // WALKING_TO_TARGET_ACTION_H
