/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Ricardo Berumen
*/

#ifndef WALKING_TO_TARGET_ACTION_H
#define WALKING_TO_TARGET_ACTION_H

#include "movement_pkg/walking_controller.h"
#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/action_node.h"


namespace BT
{
class WalkToTarget : public BT::StatefulActionNode
{
    public:
        // Constructor
        explicit WalkToTarget(const std::string& name, const NodeConfig& config);
        ~WalkToTarget() override;
        static BT::PortsList providedPorts();

        // The method that is going to be executed by the thread
        NodeStatus onStart() override;
        NodeStatus onRunning() override;
        void onHalted() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<utils> utils_;
        std::shared_ptr<WalkingController> walking_controller_;    
    //  Auxiliar methods
        void walkTowardsTarget(double head_pan_angle, double head_tilt_angle);
        void writeHeadJoint(double ang_value);
        double calculateDistance(double head_tilt);
        double calculateTilt(double remaining_distance);

        // ROS variable
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr write_joint_pub_;
        rclcpp::Time prev_time_walk_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        sensor_msgs::msg::JointState write_msg_;


        // Variables
        double walked_distance = 0.0;
        double accum_rotation = 0.0;
        double prev_delta_angle = 0.0;
        double head_pan_angle_;
        double head_tilt_angle_;
        double fb_move;
        double rl_angle;
        double distance_to_walk;
        const double distance_to_kick_ = 0.30; //0.0;  // 0.30;  // 0.22
        const double CAMERA_HEIGHT_ = 0.46;
        const double hip_pitch_offset_ = 0.12217305; //7°
        bool walkingSucced = false;
        bool walkLimitReach = false;
        const double walk_thresh  = 0.80;
        std_msgs::msg::String walk_command;
        //ros::Time prev_time_walk_ = ros::Time::now();
};
}  // namespace BT

#endif  // WALKING_TO_TARGET_ACTION_H
