/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_target_action.h"

namespace BT{
WalkToTarget::WalkToTarget(
    const std::string& name,
    const BT::NodeConfig& config) 
: StatefulActionNode(name, config)
{
        //node_ = rclcpp::Node::make_shared("simple_walk_action");
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("SimpleWalk: missing [node] in blackboard");
}
    utils_ = std::make_shared<utils>(node_);
    walking_controller_ = std::make_shared<WalkingController>(node_);
    RCLCPP_INFO(node_->get_logger(), "WalkToTarget constructed");
    write_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/robotis_" + std::to_string(robot_id) + "/set_joint_states", 10);    

}

WalkToTarget::~WalkToTarget() {}

NodeStatus BT::WalkToTarget::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[WalkToTarget] START");
    return NodeStatus::RUNNING;
}

NodeStatus BT::WalkToTarget::onRunning()
{
        // Perform action...
    walked_distance = 0;  // Resets in each cycle
    
    head_pan_angle_ = getHeadPan();
    head_tilt_angle_ = getHeadTilt();

    utils_->setModule("walking_module");
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[WalkToTarget] Walking to target..."
    ); 

    walkTowardsTarget(head_pan_angle_, head_tilt_angle_);

        if (walkingSucced)
        {
            RCLCPP_INFO(node_->get_logger(), "[WalkToTarget] Walk to target SUCCESS");
            return BT::NodeStatus::IDLE;
        }
        else if (walkLimitReach)
        {
            RCLCPP_INFO(node_->get_logger(), "[WalkToTarget] Walk to target HIT THRESHOLD");
            return BT::NodeStatus::FAILURE;
        }

        if (error_d){
            return BT::NodeStatus::IDLE;
        }
        
        return BT::NodeStatus::RUNNING;
}

void WalkToTarget::walkTowardsTarget(double head_pan_angle, double head_tilt_angle)
{
    double distance_to_ball = calculateDistance(head_tilt_angle);
    RCLCPP_INFO(node_->get_logger(), "[WalkToTarget] dist to ball: %f   ang to ball: %f", distance_to_ball, head_pan_angle);
    while (ros::ok())
    {
        ros::Time curr_time_walk = ros::Time::now();
        ros::Duration dur_walk = curr_time_walk - prev_time_walk_;
        
        if (dur_walk.toSec() == curr_time_walk.toSec())
        {
            prev_time_walk_ = curr_time_walk;
            return;
        }

        double delta_time_walk = dur_walk.toSec();
        prev_time_walk_ = curr_time_walk;
    
        if (distance_to_ball < 0)
        {
            distance_to_ball *= (-1);
        }
    
        double distance_to_walk = distance_to_ball - distance_to_kick_;
        double delta_distance = distance_to_walk - walked_distance;
        ROS_COLORED_LOG("walked dist: %f", ORANGE, false, walked_distance);

        double remaining_distance = distance_to_ball - walked_distance;
        double new_tilt = calculateTilt(remaining_distance);  // rad
    
        if (walked_distance >= distance_to_walk)
        {
            stopWalking();
            walkingSucced = true;
            writeHeadJoint(new_tilt);
            return;
        }
        else if (walked_distance >= walk_thresh)
        {
            stopWalking();
            walkLimitReach = true;
            writeHeadJoint(new_tilt);
            return;
        }
        
        else if (walked_distance >= walk_thresh)
        {
            ROS_COLORED_LOG("walked dist: %f, reached threshold: %f", ORANGE, false, walked_distance, walk_thresh);
            stopWalking();
            walkLimitReach = true;
            return;
        }

        double delta_angle = head_pan_angle - accum_rotation;

        // checking sign chance to avoid oscillation  ||  angle between a range of error
        if (delta_angle * prev_delta_angle < 0 || std::abs(delta_angle) < 0.01)
        {
            delta_angle = 0.0;  // Stop turning
        }

        prev_delta_angle = delta_angle;
    
        double fb_move = 0.0, rl_angle = 0.0;
    
        // std::cout << walked_distance << std::endl;
        // std::cout << distance_to_walk - walked_distance << std::endl;
        // std::cout << current_x_move_ << std::endl;
        // std::cout << delta_time_walk << std::endl;
    
        calcFootstep(delta_distance, delta_angle, delta_time_walk, fb_move, rl_angle);  // pan = 0
        ROS_COLORED_LOG("curr dist to ball: %f   curr ang to ball: %f", CYAN, false, delta_distance, delta_angle);

        walked_distance += fabs(fb_move);
        accum_rotation += rl_angle;
        setWalkingParam(fb_move, 0, rl_angle, true);
        
        std_msgs::String command_msg;
        command_msg.data = "start";
        walk_command_pub.publish(command_msg);
        ros::Duration(0.1).sleep();
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

double WalkToTarget::calculateDistance(double head_tilt)
{
    double distance = CAMERA_HEIGHT_ * tan(M_PI * 0.5 + head_tilt - hip_pitch_offset_);
    return distance;
}

double WalkToTarget::calculateTilt(double remaining_distance)
{
    double tilt = atan2(remaining_distance, CAMERA_HEIGHT_) - M_PI * 0.5 + hip_pitch_offset_;
    return tilt;
}

void WalkToTarget::writeHeadJoint(double ang_value)
{
    if (getModule("r_knee") != "none")
    {
        setModule("none");
        ros::Duration(1).sleep();
        ROS_COLORED_LOG("Set Module to none", YELLOW, false);
    }
    write_msg_.header.stamp = ros::Time::now();
        
    // ang_value *= 0.0174533;  // DegToRad -> pi/180
  
    if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
    else if (ang_value <= -1.2217) ang_value = -1.2217;   //-70 deg
    write_msg_.name.push_back("head_tilt");
    write_msg_.position.push_back(ang_value);

    write_joint_pub_.publish(write_msg_);
}


BT::PortsList BT::WalkToTarget::providedPorts()
{
    return {};
}

void WalkToTarget::onHalted()
{
    walking_controller_->stopWalking();
    RCLCPP_INFO(node_->get_logger(), "[WalkToTarget] HALTED: Stopped walking towards target");
    error_d = true;
}

}
