/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/center_ball_action.h"


BT::CenterBall::CenterBall(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&CenterBall::WaitForTick, this);
}

BT::CenterBall::~CenterBall() {}

void BT::CenterBall::WaitForTick()
{
    while (ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        // Flow for searching ball - specially when distance to ball >= 1m
        
        while (get_status() == BT::IDLE)
        {
            set_status(BT::RUNNING);

            ball_center_position_ = getBallPosition();
            head_pan_angle_ = getHeadPan();
            head_tilt_angle_ = getHeadTilt();
            angle_mov_x_ = head_pan_angle_ * 57.2958;   // RadToDeg -> 180/pi
            angle_mov_y_ = head_tilt_angle_ * 57.2958;  // RadToDeg -> 180/pi
            xerror_ = (320 - ball_center_position_.x) * 0.21875;  // 70 / 320
            yerror_ = (240 - ball_center_position_.y) * 0.29166;  // 70 / 240

            if (xerror_ >= 11 || xerror_ <= -11)
            {
                xerror_ = xerror_ * M_PI / 180;
        
                if (xerror_ < 0)
                {
                  angle_mov_x_ -= 1;
                }
                else
                {
                  angle_mov_x_ += 1;
                }
                writeHeadJoint(angle_mov_x_, true);
                //walkTowardsBall(current_ball_pan, head_tilt);
            }
            else if (yerror_ >= 20 || yerror_ <= -20)
            {
                yerror_ = yerror_ * M_PI / 180;
        
                if (yerror_ < 0)
                {
                  angle_mov_y_ -= 1;
                }
                else
                {
                  angle_mov_y_ += 1;
                }
                writeHeadJoint(angle_mov_y_, false);
            }
            else
            {
                ROS_SUCCESS_LOG("Ball IN CENTER! Starting walking process!");
                set_status(BT::SUCCESS);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::CenterBall::writeHeadJoint(double ang_value, bool is_pan)
{
    setModule("direct_control_module");
    write_msg_;
    write_msg_.header.stamp = ros::Time::now();
        
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
    write_joint_pub_.publish(write_msg_);
}

void BT::CenterBall::Halt()
{
    set_status(BT::HALTED);
    ROS_COLORED_LOG("CenterBall HALTED: Stopped walking.", DEFAULT, false);
}
