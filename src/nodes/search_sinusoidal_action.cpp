/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
        Ricardo Berumen
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
	
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("SimpleWalk: missing [node] in blackboard");
}
    utils_ = std::make_shared<utils>(node_);
    robot_id = utils_->robot_id;
    RCLCPP_INFO(node_->get_logger(), "SearchSinusoidal constructed");
    write_joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 10);
}
BT::SearchBallSinusoidal::~SearchBallSinusoidal() {}

NodeStatus SearchBallSinusoidal::onStart()
{   
    const rclcpp::Time now = node_->now();
    if ((now - last_active_).seconds() > 1.0)
    {
        std::string current = utils_->getModule("r_knee");
        RCLCPP_INFO(node_->get_logger(), "[SearchBallSinusoidal] Current module: %s", current.c_str());

        if (current != "direct_control_module") {
            if (current == "walking_module" || current == "base_module") {
                utils_->setModule("none");
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
            }
            utils_->setModule("direct_control_module");
            for (int i = 0; i < 10 && utils_->getModule("r_knee") != "direct_control_module"; ++i) {
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }
        }
        last_pub_ = now;
        RCLCPP_INFO(node_->get_logger(), "[SearchBallSinusoidal] START");
    }
    last_active_ = now;

    return NodeStatus::RUNNING;
}

NodeStatus SearchBallSinusoidal::onRunning()
{
    const rclcpp::Time now = node_->now();
    last_active_ = now;

    const double since_pub = (now - last_pub_).seconds();
    if (since_pub < kPublishPeriod) {
        return NodeStatus::SUCCESS;
    }
    last_pub_ = now;

    t_ += kOmega * std::min(since_pub, 0.2);  // limit to 20 Hz
    x_target_ = 60 * sin(t_);
    y_target_ = 15 * cos(t_ * 5) - 30;

    rightLeft = turn_cnt_ % 2;

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
        last_active_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        return NodeStatus::FAILURE;
    }

    writeHeadJoint(x_target_, y_target_);
    // rclcpp::sleep_for(std::chrono::milliseconds(500)); // return to 500 after fixing
    
    // ROS_SUCCESS_LOG("Searching Ball!");
    return NodeStatus::SUCCESS; // ?



}

void BT::SearchBallSinusoidal::writeHeadJoint(double ang_valueX, double ang_valueY)
{
    write_msg.name.clear();      // clean msg
    write_msg.position.clear();  
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
    last_active_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}
} //namespace BT