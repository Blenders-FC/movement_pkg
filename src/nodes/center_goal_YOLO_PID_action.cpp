/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/center_goal_YOLO_PID_action.h"


BT::CenterGoalYOLOPID::CenterGoalYOLOPID(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    goal_fts_pub_ = nh.advertise<blenders_msgs::GoalParams>("/robotis_" + std::to_string(robot_id) + "/robot_pose/goal_params", 0);
    centering_goal_pub_ = nh.advertise<std_msgs::Bool>("/robotis_" + std::to_string(robot_id) + "/robot_pose/centering_goal", 0);
    thread_ = std::thread(&CenterGoalYOLOPID::WaitForTick, this);
}

BT::CenterGoalYOLOPID::~CenterGoalYOLOPID() {}
void BT::CenterGoalYOLOPID::WaitForTick()
{
    // double Kp = 2.0, Ki = 0.0, Kd = 0.1; // Tune these
    double Kp_pan = 0.4, Ki_pan = 0.005, Kd_pan = 0.3;
    double Kp_tilt = 0.4, Ki_tilt = 0.005, Kd_tilt = 0.2;
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
            centering_goal_msg_.data = true;
            centering_goal_pub_.publish(centering_goal_msg_);

            ball_center_position_ = getBallPosition();

            if ((ball_center_position_.x == 999 && ball_center_position_.x == 0) || (ball_center_position_.y == 999 && ball_center_position_.y == 0))
            {
                ROS_COLORED_LOG("BALL NOT detected", RED, false);
                set_status(BT::FAILURE);
                break;
            }

            head_pan_angle_ = getHeadPan();
            head_tilt_angle_ = getHeadTilt();
            angle_mov_x_ = head_pan_angle_;  // rad   //* 57.2958;   // RadToDeg
            angle_mov_y_ = head_tilt_angle_; // rad   //* 57.2958;   // RadToDeg

            double error_pan = (320 - ball_center_position_.x) * X_PIXEL_TO_DEG * deg_to_rad;   // pixToDeg -> degToRad
            double error_tilt = (240 - ball_center_position_.y) * Y_PIXEL_TO_DEG * deg_to_rad;  // pixToDeg -> degToRad

            if (fabs(error_pan) < error_limit_x_ && fabs(error_tilt) < error_limit_y_)
            {
                ROS_SUCCESS_LOG("Goal IN CENTER! Calculating init pose");
                goal_msg_.distance.data = calculateDistance(head_tilt_angle_);
                goal_msg_.angle.data = head_pan_angle_;
                goal_fts_pub_.publish(goal_msg_);
                centering_goal_msg_.data = false;
                centering_goal_pub_.publish(centering_goal_msg_);
                set_status(BT::SUCCESS);
                break;
            }

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

            double output_pan = Kp_pan * error_pan + Ki_pan * integral_pan + Kd_pan * derivative_pan;
            double output_tilt = Kp_tilt * error_tilt + Ki_tilt * integral_tilt + Kd_tilt * derivative_tilt;

            angle_mov_x_ += output_pan;   // rad         // * 57.2958;   // rad to deg
            angle_mov_y_ += output_tilt;  // rad         // * 57.2958;  // rad to deg

            prev_error_pan = error_pan;
            prev_error_tilt = error_tilt;

            writeHeadJoint(angle_mov_x_, angle_mov_y_, true);
            rate.sleep();
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::CenterGoalYOLOPID::writeHeadJoint(double ang_valueX, double ang_valueY, bool ang_in_rad)
{
    // if (getModule("head_tilt") != "direct_control_module")
    // {
    //     setModule("direct_control_module");
    //     ros::Duration(1).sleep();
    //     ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
    // }
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

double BT::CenterGoalYOLOPID::calculateDistance(double head_tilt)
{
    double distance = CAMERA_HEIGHT_ * tan(M_PI * 0.5 + head_tilt - hip_pitch_offset_);
    return distance;
}

void BT::CenterGoalYOLOPID::Halt()
{
    set_status(BT::HALTED);
    ROS_COLORED_LOG("CenterGoalYOLOPID HALTED: Stopped centering ball", ORANGE, false);
}
