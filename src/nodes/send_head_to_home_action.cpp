/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
*/

#include "movement_pkg/nodes/send_head_to_home_action.h"

namespace BT {
    HeadToHome::HeadToHome(
        const std::string& name, 
        const BT::NodeConfig& config)
    : StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("send_head_to_home");
        RCLCPP_INFO(node_->get_logger(), "SearchSinusoidal constructed");
        write_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    }

BT::HeadToHome::~HeadToHome() {}

NodeStatus HeadToHome::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] START");

    return NodeStatus::RUNNING;
}

NodeStatus BT::HeadToHome::onRunning()
{

    // set_status(BT::RUNNING);
    setModule("direct_control_module");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    //ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] Set Module to direct_control_module");

    writeHeadJoint(0, true);
    writeHeadJoint(-10, false);
    //ROS_COLORED_LOG("New tilt angle position from head2home: %f", TEAL, false, -10);
    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] New tilt angle position from head2home: %d", -10);
    rclcpp::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] Head in home position!");
    return NodeStatus::SUCCESS;

}

void BT::HeadToHome::writeHeadJoint(double ang_value, bool is_pan)
{
    write_msg_.header.stamp = node_->get_clock()->now(); 
        
    ang_value *= 0.0174533;  // DegToRad -> pi/180
  
    if (is_pan){
      if (ang_value >= 1.2217) ang_value = 1.2217;            //70 deg
      else if (ang_value <= -1.2217) ang_value = -1.2217;     //-70 deg
      write_msg_.name.push_back("head_pan");
      write_msg_.position.push_back(ang_value);
    }else{
      if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
      else if (ang_value <= -1.2217) ang_value = -1.2217;   //-70 deg
      write_msg_.name.push_back("head_tilt");
      write_msg_.position.push_back(ang_value);
    }
    write_joint_pub_->publish(write_msg_);
}

BT::PortsList BT::HeadToHome::providedPorts()
{
    return {};
}

void BT::HeadToHome::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] HALTED: Stopped head movement");
}
}