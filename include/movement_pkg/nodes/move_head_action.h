/*
    Authors:
         Vicman Gil
*/

#ifndef MOVE_HEAD_ACTION_H
#define MOVE_HEAD_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class MoveHead : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit MoveHead(std::string name);
        ~MoveHead();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:       

        void writeHeadJoint(double ang_value, bool is_pan);

        // ROS variable
        ros::Publisher write_joint_pub_;

        // Variables
        bool is_pan;
        double angle;
        double angle_mov_x_ = 0;
        double angle_mov_y_;
        double head_pan_angle_;
        double head_tilt_angle_;
        double limit_x_error_;
        double limit_y_error_;
        double ang_value = -1;         
        int turn_cnt_ = 0;
        sensor_msgs::JointState write_msg_;
};
}  // namespace BT

#endif  // MOVE_HEAD_ACTION_H
