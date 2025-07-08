/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/center_ball_YOLOPID.h"


BT::CenterBallYOLOPID::CenterBallYOLOPID(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&CenterBallYOLOPID::WaitForTick, this);
}

BT::CenterBallYOLOPID::~CenterBallYOLOPID() {}
void BT::CenterBallYOLOPID::WaitForTick()
{
    // double Kp = 2.0, Ki = 0.0, Kd = 0.1; // Tune these
    double Kp = 1.0, Ki = 0.0, Kd = 0.3; // Tune these
    double integral_pan = 0, integral_tilt = 0;
    double prev_error_pan = 0, prev_error_tilt = 0;
    ros::Rate rate(30); // 30 Hz
    double dt = 1.0 / 30.0;

    while (ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "CWait_enterBallYOLOPID");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_CenterBallYOLOPID");

        set_status(BT::RUNNING);

        while (get_status() == BT::RUNNING)
        {
            ball_center_position_ = getBallPosition();
            head_pan_angle_ = getHeadPan();
            head_tilt_angle_ = getHeadTilt();
            angle_mov_x_ = head_pan_angle_ * 57.2958;   // RadToDeg
            angle_mov_y_ = head_tilt_angle_ * 57.2958;  // RadToDeg

            double error_pan = (320 - ball_center_position_.x) * 0.21875 * M_PI / 180;  // 70 / 320 -> pixToGrad -> Grad to rad
            double error_tilt = (240 - ball_center_position_.y) * 0.29166 * M_PI / 180; // 70 / 240 -> pixToGrad -> Grad to rad

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

            angle_mov_x_ += output_pan * 57.2958;   // rad to deg
            angle_mov_y_ += output_tilt * 57.2958;  // rad to deg

            prev_error_pan = error_pan;
            prev_error_tilt = error_tilt;

            if (fabs(error_pan) < 0.05 && fabs(error_tilt) < 0.05)
            {
                ROS_SUCCESS_LOG("Ball IN CENTER! Starting walking process!");
                set_status(BT::SUCCESS);
                break;
            }

            writeHeadJoint(angle_mov_x_, true);
            writeHeadJoint(angle_mov_y_, false);

            rate.sleep();
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::CenterBallYOLOPID::writeHeadJoint(double ang_value, bool is_pan)
{
    if (getModule("r_knee") != "direct_control_module")
    {
        setModule("direct_control_module");
        ros::Duration(1).sleep();
        ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
    }
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

void BT::CenterBallYOLOPID::Halt()
{
    set_status(BT::HALTED);
    ROS_COLORED_LOG("CenterBallYOLOPID HALTED: Stopped centering ball", ORANGE, false);
}

