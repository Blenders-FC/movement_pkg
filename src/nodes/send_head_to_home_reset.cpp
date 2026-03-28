/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
*/

#include "movement_pkg/nodes/send_head_to_home_reset.h"


namespace BT {
    HeadToHomeReset::HeadToHomeReset(
        const std::string& name, 
        const BT::NodeConfig& config)
    : StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("send_head_to_home_reset");
        RCLCPP_INFO(node_->get_logger(), "[HeadToHomeReset] constructed");
        write_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    }

BT::HeadToHomeReset::~HeadToHomeReset() {}

NodeStatus HeadToHomeReset::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[HeadToHomeReset] START");

    return NodeStatus::RUNNING;
}

NodeStatus BT::HeadToHomeReset::onRunning()
{
    RCLCPP_INFO(node_->get_logger(), "[HeadToHomeReset] Set Module to direct_control_module");

    setModule("direct_control_module");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    writeHeadJoint(0, true);
    writeHeadJoint(-10, false);

    rclcpp::sleep_for(std::chrono::milliseconds(2000));

    RCLCPP_INFO(node_->get_logger(), "[HeadToHome] Head in home position! Resetting Counter");

    m_turncnt.turncnt = 0;
    config().blackboard->set("m_turncnt", m_turncnt);

    return NodeStatus::SUCCESS;
}

void BT::HeadToHomeReset::writeHeadJoint(double ang_value, bool is_pan)
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

void BT::HeadToHomeReset::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[HeadToHomeReset] HALTED: Stopped head movement");
}
}  // namespace BT