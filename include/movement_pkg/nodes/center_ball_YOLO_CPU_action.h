/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CENTER_BALL_YOLO_CPU_ACTION_H
#define CENTER_BALL_YOLO_CPU_ACTION_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class CenterBallYOLOCPU : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit CenterBallYOLOCPU(std::string name);
        ~CenterBallYOLOCPU();

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
        geometry_msgs::Point ball_center_position_;
        bool head_direction_ = true;
        double head_pan_angle_;
        double head_tilt_angle_;
        double angle_mov_x_;
        double angle_mov_y_;
        double xerror_;
        double yerror_;
        sensor_msgs::JointState write_msg_;
};
}  // namespace BT

#endif  // CENTER_BALL_YOLO_CPU_ACTION_H
