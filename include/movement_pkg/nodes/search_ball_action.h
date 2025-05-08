/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef SEARCH_BALL_ACTION_H
#define SEARCH_BALL_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class SearchBall : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit SearchBall(std::string name);
        ~SearchBall();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:
        //  Auxiliar methods
        void writeHeadJoint(double ang_value, bool is_pan);

        // ROS variable
        ros::Publisher write_joint_pub_;

        // Variables
        bool head_direction_ = true;
        double angle_mov_x_ = 0;
        double angle_mov_y_;
        double head_pan_angle_;
        double head_tilt_angle_;
        double limit_x_error_;
        double limit_y_error_;
        int turn_cnt_ = 0;
        sensor_msgs::JointState write_msg_x_;
        sensor_msgs::JointState write_msg_y_;
};
}  // namespace BT

#endif  // SEARCH_BALL_ACTION_H
