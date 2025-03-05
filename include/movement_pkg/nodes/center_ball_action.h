/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CENTER_BALL_ACTION_H
#define CENTER_BALL_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class CenterBall : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit CenterBall(std::string name);
        ~CenterBall();

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
        double angle_mov_x_;
        double angle_mov_y_;
        double xerror_;
        double yerror_;
        sensor_msgs::JointState write_msg_;
};
}  // namespace BT

#endif  // CENTER_BALL_ACTION_H
