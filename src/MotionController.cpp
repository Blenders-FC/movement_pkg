/* 
Authors: Pedro Deniz
           Marlene Cobian 
*/

#include "movement_pkg/MotionController.h"


BallDetectedCondition::BallDetectedCondition(std::string name) : BT::ConditionNode(name)
{
    ball_sub = nh.subscribe("/robotis/ball_center", 1, &BallDetectedCondition::callbackBallCenter, this);
}


void BallDetectedCondition::Execute()
{
    setStatus(ball_detected ? BT::SUCCESS : BT::FAILURE);
}


void BallDetectedCondition::callbackBallCenter(const geometry_msgs::Point::ConstPtr &msg)
{
    ball_detected = (msg->x != 999 && msg->y != 999);
}


MoveToBallAction::MoveToBallAction(std::string name) : BT::ActionNode(name)
{
    walk_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 1);
}


void MoveToBallAction::Execute()
{
    ROS_INFO("Moving towards the ball...");
    std_msgs::String command_msg;
    command_msg.data = "start";
    walk_command_pub.publish(command_msg);
    setStatus(BT::SUCCESS);
}


SearchBallAction::SearchBallAction(std::string name) : BT::ActionNode(name)
{
    head_scan_pub = nh.advertise<std_msgs::String>("/robotis/head_control/scan_command", 1);
}


void SearchBallAction::Execute()
{
    ROS_INFO("Searching for the ball...");
    std_msgs::String scan_msg;
    scan_msg.data = "scan";
    head_scan_pub.publish(scan_msg);
    setStatus(BT::SUCCESS);
}


HeadTrackingAction::HeadTrackingAction(std::string name) : BT::ActionNode(name)
{
    head_control_pub = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 1);
}


void HeadTrackingAction::Execute()
{
    ROS_INFO("Adjusting head position for tracking");
    sensor_msgs::JointState head_msg;
    head_msg.name.push_back("head_pan");
    head_msg.position.push_back(angle_mov_x);
    head_msg.name.push_back("head_tilt");
    head_msg.position.push_back(angle_mov_y);
    head_control_pub.publish(head_msg);
    setStatus(BT::SUCCESS);
}


FallRecoveryCondition::FallRecoveryCondition(std::string name) : BT::ConditionNode(name)
{
    imu_sub = nh.subscribe("/robotis/imu", 1, &FallRecoveryCondition::callbackImu, this);
}


void FallRecoveryCondition::Execute()
{
    if (present_pitch_ > FALL_FORWARD_LIMIT || present_pitch_ < FALL_BACK_LIMIT)
        setStatus(BT::SUCCESS);
    else
        setStatus(BT::FAILURE);
}


void FallRecoveryCondition::callbackImu(const sensor_msgs::Imu::ConstPtr &msg)
{
    present_pitch_ = msg->orientation.x; // Example: replace with actual pitch calculation
}


KickAction::KickAction(std::string name) : BT::ActionNode(name)
{
    kick_pub = nh.advertise<std_msgs::String>("/robotis/kick_command", 1);
}


void KickAction::Execute()
{
    ROS_INFO("Kicking the ball");
    std_msgs::String kick_msg;
    kick_msg.data = "kick";
    kick_pub.publish(kick_msg);
    setStatus(BT::SUCCESS);
}
