/*
    Authors:
        Pedro Deniz
        Marlene Cobian

        Victor Gil
*/

#include "movement_pkg/nodes/center_ball_YOLO_PID_action.h"

namespace BT{
CenterBallYOLOPID::CenterBallYOLOPID(
    const std::string& name, 
    const BT::NodeConfig& config
) 
: StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("center_ball_YOLO_PID_action");
    RCLCPP_INFO(node_->get_logger(), "CenterBallYOLOPID constructed");
    write_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(utils_->robot_id) + "/direct_control/set_joint_states", 10);    
}

BT::CenterBallYOLOPID::~CenterBallYOLOPID() {}

NodeStatus BT::CenterBallYOLOPID::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[CenterBallYOLOPID] START");
    return NodeStatus::RUNNING;
}

NodeStatus BT::CenterBallYOLOPID::onRunning()
{
    // double Kp = 2.0, Ki = 0.0, Kd = 0.1; // Tune these
    double Kp = 1.0, Ki = 0.0, Kd = 0.3; // Tune these
    double integral_pan = 0, integral_tilt = 0;
    double prev_error_pan = 0, prev_error_tilt = 0;
    rclcpp::Rate rate(30); // 30 Hz
    double dt = 0.033333;

    ball_center_position_ = getBallPosition();

    if ((ball_center_position_.x == 999 || ball_center_position_.x == 0) || (ball_center_position_.y == 999 || ball_center_position_.y == 0))
    {
        RCLCPP_WARN(node_->get_logger(), "BALL NOT detected. Not able to center.");
        return NodeStatus::FAILURE;
    }
    head_pan_angle_ = getHeadPan();
    head_tilt_angle_ = getHeadTilt();
    angle_mov_x_ = head_pan_angle_;  // rad   //* 57.2958;   // RadToDeg
    angle_mov_y_ = head_tilt_angle_; // rad   //* 57.2958;   // RadToDeg

    double error_pan = (320 - ball_center_position_.x) * X_PIXEL_TO_DEG * deg_to_rad;   // pixToDeg -> degToRad
    double error_tilt = (240 - ball_center_position_.y) * Y_PIXEL_TO_DEG * deg_to_rad;  // pixToDeg -> degToRad

    integral_pan += error_pan * dt;
    integral_tilt += error_tilt * dt;

    // Anti-windup clamp
    double integral_limit = 0.2;
    if (integral_pan > integral_limit) integral_pan = integral_limit;
    if (integral_pan < -integral_limit) integral_pan = -integral_limit;
    if (integral_tilt > integral_limit) integral_tilt = integral_limit;
    if (integral_tilt < -integral_limit) integral_tilt = -integral_limit;

    double derivative_pan = (error_pan - prev_error_pan) / dt;
    double derivative_tilt = (error_tilt - prev_error_tilt) / dt;

    double output_pan = Kp * error_pan + Ki * integral_pan + Kd * derivative_pan;
    double output_tilt = Kp * error_tilt + Ki * integral_tilt + Kd * derivative_tilt;

    angle_mov_x_ += output_pan;   // rad         // * 57.2958;   // rad to deg
    angle_mov_y_ += output_tilt;  // rad         // * 57.2958;  // rad to deg

    prev_error_pan = error_pan;
    prev_error_tilt = error_tilt;

    if (fabs(error_pan) < error_limit_ && fabs(error_tilt) < error_limit_)
    {
        RCLCPP_INFO(node_->get_logger(), "Ball IN CENTER! Starting walking process!");
        return NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "Centering camera on ball...");
    writeHeadJoint(angle_mov_x_, angle_mov_y_, true);
    rate.sleep();
    return NodeStatus::RUNNING;
}

void BT::CenterBallYOLOPID::writeHeadJoint(double ang_valueX, double ang_valueY, bool ang_in_rad)
{
    if (utils_->getModule("head_tilt") != "direct_control_module")
    {
        utils_->setModule("direct_control_module");
        rclcpp::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(node_->get_logger(), "Set Module to direct_control_module");
    }

    write_msg_.header.stamp = node_->get_clock()->now();  
    
    if (!ang_in_rad)
    {
        ang_valueX *= deg_to_rad;  // DegToRad -> pi/180
        ang_valueY *= deg_to_rad;  // DegToRad -> pi/180
    }
    
    if (ang_valueX >= PAN_MAX_RAD) ang_valueX = PAN_MAX_RAD;            //70 deg
    else if (ang_valueX <= PAN_MIN_RAD) ang_valueX = PAN_MIN_RAD;     //-70 deg
    write_msg_.name.push_back("head_pan");
    write_msg_.position.push_back(ang_valueX);    

    if (ang_valueY >= TILT_MAX_RAD) ang_valueY = TILT_MAX_RAD;        //20 deg
    else if (ang_valueY <= TILT_MIN_RAD) ang_valueY = TILT_MIN_RAD;   //-70 deg
    write_msg_.name.push_back("head_tilt");
    write_msg_.position.push_back(ang_valueY);

    write_joint_pub_->publish(write_msg_);
}

BT::PortsList BT::CenterBallYOLOPID::providedPorts()
{
    return {};
}

void CenterBallYOLOPID::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[SearchBallSinusoidal] HALTED: Stopped ball searching");
}

}  // namespace BT