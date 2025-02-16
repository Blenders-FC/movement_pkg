/* 
Authors: Pedro Deniz
           Marlene Cobian 
*/

#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <behavior_tree_core/ActionNode.h>
#include <behavior_tree_core/ConditionNode.h>


class BallDetectedCondition : public BT::ConditionNode
{
public:
    BallDetectedCondition(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Subscriber ball_sub;
    bool ball_detected = false;
    void callbackBallCenter(const geometry_msgs::Point::ConstPtr &msg);
};


class MoveToBallAction : public BT::ActionNode
{
public:
    MoveToBallAction(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Publisher walk_command_pub;
};


class SearchBallAction : public BT::ActionNode
{
public:
    SearchBallAction(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Publisher head_scan_pub;
};


class HeadTrackingAction : public BT::ActionNode
{
public:
    HeadTrackingAction(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Publisher head_control_pub;
    double angle_mov_x, angle_mov_y;
};


class FallRecoveryCondition : public BT::ConditionNode
{
public:
    FallRecoveryCondition(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    double present_pitch_;
    const double FALL_FORWARD_LIMIT = 55;
    const double FALL_BACK_LIMIT = -55;
    void callbackImu(const sensor_msgs::Imu::ConstPtr &msg);
};


class KickAction : public BT::ActionNode
{
public:
    KickAction(std::string name);
    void Execute() override;

private:
    ros::NodeHandle nh;
    ros::Publisher kick_pub;
};


#endif // MOTIONCONTROLLER_H
