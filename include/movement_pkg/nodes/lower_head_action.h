/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef LOWER_HEAD_ACTION_H
#define LOWER_HEAD_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class LowerHead : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit LowerHead(std::string name);
        ~LowerHead();

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
        double ang_value = -1;         
        int turn_cnt_ = 0;
        sensor_msgs::JointState write_msg_x_;
        sensor_msgs::JointState write_msg_y_;
};
}  // namespace BT

#endif  // LOWER_HEAD_ACTION_H
