/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/search_ball_action.h"


BT::SearchBall::SearchBall(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&SearchBall::WaitForTick, this);
}

BT::SearchBall::~SearchBall() {}

void BT::SearchBall::WaitForTick()
{
    while (ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_searchball");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");
        
        while (get_status() == BT::IDLE)
        {
            turn_cnt_ = 0;
    
            set_status(BT::RUNNING);
            setModule("direct_control_module");
            ros::Duration(1.0).sleep();
            ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
            // Flow for searching ball - specially when distance to ball >= 1m

            head_pan_angle_ = getHeadPan();
            angle_mov_x_ = head_pan_angle_ * 57.2958;  // RadToDeg -> 180/pi

            if (head_direction_ && angle_mov_x_ <= 70)
            {
                angle_mov_x_ += 5;
                ROS_COLORED_LOG("New angle position: %f", CYAN, false, angle_mov_x_);
                writeHeadJoint(angle_mov_x_, true);
                ros::Duration(1.0).sleep();
                if (angle_mov_x_ >= 70) head_direction_ = false;
            }
            else if (!head_direction_ && angle_mov_x_ >= -70)
            {
                angle_mov_x_ -= 5;
                writeHeadJoint(angle_mov_x_, true);
                ros::Duration(1.0).sleep();
                if (angle_mov_x_ <= -70)
                {
                    head_direction_ = true;
                    turn_cnt_ += 1;
                    if (turn_cnt_ == 1)
                    {
                        angle_mov_y_ = -50;
                        writeHeadJoint(angle_mov_y_, false);
                    }
                    else if (turn_cnt_ == 2) 
                    {
                        turn_cnt_ = 0;
                        // turn2search(9);
                        ROS_COLORED_LOG("Couldn't find ball! Changing the search position...", YELLOW, false);
                        set_status(BT::FAILURE);
                    }
                }
            }
        }
    }
}

void BT::SearchBall::writeHeadJoint(double ang_value, bool is_pan)
{
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
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SearchBall HALTED: Stopped walking.");
}
