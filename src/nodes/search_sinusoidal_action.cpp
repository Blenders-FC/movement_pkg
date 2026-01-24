/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
*/

#include "movement_pkg/nodes/search_sinusoidal_action.h"


using namespace std::chrono_literals;

namespace BT{
SearchBallSinusoidal::SearchBallSinusoidal(
	const std::string& name, 
	const BT::NodeConfig& config) 
: StatefulActionNode(name, config)
{
    //type_ = BT::ACTION_NODE;
    //write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    //thread_ = std::thread(&SearchBallSinusoidal::WaitForTick, this);
	node_ = rclcpp::Node::make_shared("search_sinusoidal_action");
    RCLCPP_INFO(node_->get_logger(), "SearchSinusoidal constructed");
    write_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 10);
}

BT::SearchBallSinusoidal::~SearchBallSinusoidal() {}

NodeStatus SearchBallSinusoidal::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[SearchBallSinusoidal] START");

    return NodeStatus::RUNNING;
}

NodeStatus SearchBallSinusoidal::onRunning()
{
    if (getModule("r_knee") != "direct_control_module")
            {
                setModule("direct_control_module");
                rclcpp::sleep_for(std::chrono::milliseconds(500));
                // ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
                RCLCPP_INFO(node_->get_logger(), "Set Module to direct_control_module");
            }

    t_ += 0.1;

    x_target_ = 60*sin(t_);
    y_target_ = 15*cos(t_*5) - 30;

    rightLeft = turn_cnt_%2;

    RCLCPP_INFO(node_->get_logger(), "x target: %f", x_target_);
    RCLCPP_INFO(node_->get_logger(), "y target: %f", y_target_);

    dx = x_target_ - x_target_past_;
    x_target_past_= x_target_;
    
    if (((!rightLeft) && (dx <= 0)) || ((rightLeft) && (dx >= 0))){
        turn_cnt_++;
    }

    if (turn_cnt_ >= 2) {
        turn_cnt_ = 0;
        RCLCPP_INFO(node_->get_logger(), "Couldn't find ball! Changing the search position...");
        head_direction_ = true;
        return NodeStatus::FAILURE;
    }

    writeHeadJoint(x_target_, y_target_);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // ROS_SUCCESS_LOG("Searching Ball!");
    return NodeStatus::SUCCESS; // ?



}

void BT::SearchBallSinusoidal::writeHeadJoint(double ang_valueX, double ang_valueY)
{
    write_msg.header.stamp = node_->get_clock()->now();     
    ang_valueX *= 0.0174533;  // DegToRad -> pi/180
    ang_valueY *= 0.0174533;  // DegToRad -> pi/180
  
    
    if (ang_valueX >= 1.22173) ang_valueX = 1.22173;            //70 deg
    else if (ang_valueX <= -1.22173) ang_valueX = -1.22173;     //-70 deg
    write_msg.name.push_back("head_pan");
    write_msg.position.push_back(ang_valueX);    

    if (ang_valueY >= 0.34906) ang_valueY= 0.34906;        //20 deg
    else if (ang_valueY <= -1.22173) ang_valueY = -1.22173;   //-70 deg
    write_msg.name.push_back("head_tilt");
    write_msg.position.push_back(ang_valueY);

    write_joint_pub_->publish(write_msg);
}

BT::PortsList BT::SearchBallSinusoidal::providedPorts()
{
    return {};
}

void SearchBallSinusoidal::onHalted()
{
    
    RCLCPP_INFO(node_->get_logger(), "[SearchBallSinusoidal] HALTED: Stopped ball searching");
}
} //namespace BT