/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef HEAD_TO_HOME_RESET_H
#define HEAD_TO_HOME_RESET_H

#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/action_node.h"
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>


namespace BT
{
class HeadToHomeReset : public StatefulActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit HeadToHomeReset(const std::string& name, const BT::NodeConfig& config);
        ~HeadToHomeReset();

        static BT::PortsList providedPorts();

        NodeStatus onStart() override;
        NodeStatus onRunning() override;
        void onHalted() override;

    private:

        struct TargetInfo
        {
        int turncnt;
        };
        //  Auxiliar methods
        void writeHeadJoint(double ang_value, bool is_pan);

        // ROS variable
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr write_joint_pub_;
        sensor_msgs::msg::JointState write_msg_;

        TargetInfo m_turncnt;
};
}  // namespace BT

#endif  // HEAD_TO_HOME_ACTION_H
