/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/send_head_to_home_action.h"


BT::HeadToHome::HeadToHome(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&HeadToHome::WaitForTick, this);
}

BT::HeadToHome::~HeadToHome() {}

void BT::HeadToHome::WaitForTick()
{
    while (ros::ok())
    {
        // ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_HeadToHome");
        tick_engine.Wait();
        // ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_HeadToHome");
        
        while (get_status() == BT::IDLE)
        {
            // set_status(BT::RUNNING);
            setModule("direct_control_module");
            ros::Duration(1).sleep();
            ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);

            writeHeadJoint(0, true);
            writeHeadJoint(-10, false);
            ros::Duration(1).sleep();

            ROS_SUCCESS_LOG("Head in home position!");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::HeadToHome::writeHeadJoint(double ang_value, bool is_pan)
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

void BT::HeadToHome::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("HeadToHome HALTED: Stopped search ball", "ORANGE", false, "Halted_HeadToHome");
}
