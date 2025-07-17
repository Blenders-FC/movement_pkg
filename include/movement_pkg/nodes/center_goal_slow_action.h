/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CENTER_GOAL_SLOW_ACTION_H
#define CENTER_GOAL_SLOW_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class CenterGoalSlow : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit CenterGoalSlow(std::string name);
        ~CenterGoalSlow();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:
        //  Auxiliar methods
        void writeHeadJoint(double ang_value, bool is_pan);
        double calculateDistance(double head_tilt);

        // ROS variable
        ros::Publisher write_joint_pub_;
        ros::Publisher goal_fts_pub_;

        // Variables
        geometry_msgs::Point ball_center_position_;
        bool head_direction_ = true;
        double head_pan_angle_;
        double head_tilt_angle_;
        double angle_mov_x_;
        double angle_mov_y_;
        double xerror_;
        double yerror_;
        const double CAMERA_HEIGHT_ = 0.46;
        const double hip_pitch_offset_ = 0.12217305; //7°
        sensor_msgs::JointState write_msg_;
        blenders_msgs::GoalParams goal_msg_;
};
}  // namespace BT

#endif  // CENTER_GOAL_SLOW_ACTION_H
