/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/
 
#ifndef TURN_LEFT_ACTION_H
#define TURN_LEFT_ACTION_H
 
#include <memory>
#include <string>
#include <vector>
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "behaviortree_cpp/action_node.h"
#include "movement_pkg/utils.h"
 
namespace BT
{
class TurnLeft : public BT::StatefulActionNode
{
public:
    TurnLeft(const std::string& name, const BT::NodeConfig& config);
    ~TurnLeft() override = default;
 
    static BT::PortsList providedPorts();
 
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
 
private:
    // Publishes one of the 4 phases of a turn (0..3)
    void publishStep(int step);
 
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<utils> utils_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr write_joint_pub_;
    sensor_msgs::msg::JointState write_msg_;
 
    std::vector<std::vector<float>> positions_;  // loaded in the constructor, NOT here
    int robot_id_ = 0;
 
    // State machine
    int turns_num_ = 1;
    int turn_count_ = 0;
    int step_ = 0;
    rclcpp::Time last_step_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
 
    static constexpr int kStepsPerTurn = 4;
    static constexpr double kStepPeriod = 0.1;  // s, same as the old ros::Duration(0.1)
    const double crouch_angle_ = 0.08726;       // 5 deg
    const int rows_ = 40;
};
}  // namespace BT
 
#endif  // TURN_LEFT_ACTION_H
 
