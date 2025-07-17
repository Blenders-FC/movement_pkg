/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/center_ball_YOLO_PID_action.h"


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
    double Kp_x = 0.4, Ki_x = 0.005, Kd_x = 0.3; // Tune these
    double Kp_y = 0.4, Ki_y = 0.005, Kd_y = 0.2;
    double integral_pan = 0, integral_tilt = 0;
    double prev_error_pan = 0, prev_error_tilt = 0;
    ros::Rate rate(30); // 30 Hz
    double dt = 0.033333;

    while (ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_enterBallYOLOPID");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_CenterBallYOLOPID");


        while (get_status() == BT::IDLE)
        {
            // set_status(BT::RUNNING);

            ball_center_position_ = getBallPosition();

            if ((ball_center_position_.x == 999 || ball_center_position_.x == 0) || (ball_center_position_.y == 999 || ball_center_position_.y == 0))
            {
                ROS_COLORED_LOG("BALL NOT detected. Not able to center.", RED, false);
                //set_status(BT::FAILURE);
                continue;;
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

            double output_pan = Kp_x * error_pan + Ki_x * integral_pan + Kd_x * derivative_pan;
            double output_tilt = Kp_y * error_tilt + Ki_y * integral_tilt + Kd_y * derivative_tilt;

            angle_mov_x_ += output_pan;   // rad         // * 57.2958;   // rad to deg
            angle_mov_y_ += output_tilt;  // rad         // * 57.2958;  // rad to deg

            prev_error_pan = error_pan;
            prev_error_tilt = error_tilt;

            if (fabs(error_pan) < error_limit_ && fabs(error_tilt) < error_limit_)
            {
                ROS_SUCCESS_LOG("Ball IN CENTER! Starting walking process!");
                set_status(BT::SUCCESS);
                break;
            }

            ROS_COLORED_LOG("Centering camera on ball...", ORANGE, false);

            writeHeadJoint(angle_mov_x_, angle_mov_y_, true);
            rate.sleep();
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::CenterBallYOLOPID::writeHeadJoint(double ang_valueX, double ang_valueY, bool ang_in_rad)
{
    if (getModule("head_tilt") != "direct_control_module")
    {
        setModule("direct_control_module");
        ros::Duration(1).sleep();
        ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
    }

    write_msg_.header.stamp = ros::Time::now();
    
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

    write_joint_pub_.publish(write_msg_);
}

void BT::CenterBallYOLOPID::Halt()
{
    set_status(BT::HALTED);
    ROS_COLORED_LOG("CenterBallYOLOPID HALTED: Stopped centering ball", ORANGE, false);
}

