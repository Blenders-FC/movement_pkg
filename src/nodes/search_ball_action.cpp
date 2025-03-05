/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/search_ball_action.h"


BT::SearchBall::SearchBall(std::string name) 
: ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&SearchBall::WaitForTick, this);
}

BT::SearchBall::~SearchBall() {}

void BT::SearchBall::WaitForTick()
{
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        turn_cnt_ = 0

        set_status(BT::RUNNING);

        // Flow for searching ball - specially when distance to ball >= 1m
        
        while (get_status() != BT::HALTED)
        {
            head_pan_angle_ = getHeadPan();
            angle_mov_x_ = head_pan_angle_ * 57.2958  // RadToDeg -> 180/pi

            if (head_direction_ && angle_mov_x_ <= 70)
            {
                angle_mov_x_ += 1;
                writeHeadJoint(angle_mov_x_, true);
                if (angle_mov_x_ == 70) head_direction_ = false;
            }
            else if (!head_direction_ && angle_mov_x_ >= -70)
            {
                angle_mov_x_ -= 1;
                writeHeadJoint(angle_mov_x_, true);
                if (angle_mov_x_ == -70)
                {
                    head_direction_ = true;
                    turn_cnt_ += 1;
                    if (turn_cnt_ == 1)
                    {
                        angle_mov_y_ = -50;
                        ros::Duration(1.0).sleep();
                        writeHeadJoint(angle_mov_y_, false);
                        ros::Duration(1.0).sleep();
                    }
                    else if (turn_cnt_ == 2) 
                    {
                        turn_cnt_ = 0;
                        // turn2search(9);
                        DEBUG_STDOUT(get_name() << "Walk to target SUCCESS");
                        set_status(BT::SUCCESS);
                    }
                }
            }
        }
    }
}

void BT::SearchBall::writeHeadJoint(double ang_value, bool is_pan)
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

void BT::SearchBall::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    DEBUG_STDOUT("SearchBall HALTED: Stopped walking.");
}
