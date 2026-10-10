/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/
 
#include "movement_pkg/nodes/turn_left_action.h"
 
#include <chrono>
 
BT::TurnLeft::TurnLeft(const std::string& name, const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
    if (!config.blackboard->get("node", node_)) {
        throw BT::RuntimeError("TurnLeft: missing [node] in blackboard");
    }
 
    utils_ = std::make_shared<utils>(node_);
    robot_id_ = utils_->robot_id;
 
    // Must be loaded here: utils_ is null while member initializers run
    positions_ = utils_->loadPositions();
    if (positions_.size() < static_cast<size_t>(rows_)) {
        throw BT::RuntimeError("TurnLeft: could not load StandUpPositions.txt");
    }
 
    write_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/robotis_" + std::to_string(robot_id_) + "/set_joint_states", 10);
 
    RCLCPP_INFO(node_->get_logger(), "[TurnLeft] constructed");
}
 
BT::PortsList BT::TurnLeft::providedPorts()
{
    return { BT::InputPort<int>("turns", 1, "Number of 15 deg turns") };
}
 
BT::NodeStatus BT::TurnLeft::onStart()
{
    if (!getInput<int>("turns", turns_num_) || turns_num_ < 1) {
        RCLCPP_ERROR(node_->get_logger(), "[TurnLeft] invalid [turns] port");
        return BT::NodeStatus::FAILURE;
    }
 
    RCLCPP_INFO(node_->get_logger(), "[TurnLeft] Turning left in place (%d turns)", turns_num_);
 
    if (utils_->getModule("l_knee") != "none")
    {
        utils_->setModule("none");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
 
    turn_count_ = 0;
    step_ = 0;
    last_step_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
}
 
BT::NodeStatus BT::TurnLeft::onRunning()
{
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "[TurnLeft] ROS stopped unexpectedly");
        return BT::NodeStatus::FAILURE;
    }
 
    const rclcpp::Time now = node_->now();
    if ((now - last_step_time_).seconds() < kStepPeriod) {
        return BT::NodeStatus::RUNNING;
    }
 
    publishStep(step_);
    last_step_time_ = now;
 
    if (++step_ >= kStepsPerTurn)
    {
        step_ = 0;
        if (++turn_count_ >= turns_num_)
        {
            RCLCPP_INFO(node_->get_logger(), "[TurnLeft] Turning left finished successfully");
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::RUNNING;
}
 
void BT::TurnLeft::onHalted()
{
    step_ = 0;
    turn_count_ = 0;
    RCLCPP_INFO(node_->get_logger(), "[TurnLeft] HALTED: stopped turning in place");
}
 
// turning 15 deg in each cycle (4 phases)
void BT::TurnLeft::publishStep(int step)
{
    write_msg_.header.stamp = node_->now();
    write_msg_.name.clear();
    write_msg_.position.clear();
 
    auto add = [this](const std::string& joint, double value) {
        write_msg_.name.push_back(joint);
        write_msg_.position.push_back(value);
    };
 
    const auto& last = positions_[rows_ - 1];
 
    switch (step)
    {
    case 0:  // left foot up
        add("l_ank_pitch", 0.7091);
        add("l_knee", 1.4131);
        add("l_hip_pitch", -0.7091 - crouch_angle_);
        add("r_hip_yaw", 0.1746 * 1.5);
        add("l_hip_yaw", -0.1746 * 1.5);
        add("l_hip_roll", -0.0873);
        add("r_hip_roll", 0.0873);
        add("l_ank_roll", -0.0873);
        add("r_ank_roll", 0.0873);
        break;
    case 1:  // left foot down
        add("l_ank_pitch", last[3]);
        add("l_knee", last[4]);
        add("l_hip_pitch", last[5] - crouch_angle_);
        break;
    case 2:  // right foot up
        add("r_ank_pitch", -0.7091);
        add("r_knee", -1.4131);
        add("r_hip_pitch", 0.7091 + crouch_angle_);
        add("r_hip_yaw", 0.0);
        add("l_hip_yaw", 0.0);
        break;
    case 3:  // right foot down
        add("r_ank_pitch", last[0]);
        add("r_knee", last[1]);
        add("r_hip_pitch", last[2] + crouch_angle_);
        break;
    }
 
    write_joint_pub_->publish(write_msg_);
}
 
