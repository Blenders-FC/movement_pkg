/*
    Authors:
        Vicman Gil
*/

#include "movement_pkg/nodes/move_head_action.h"


BT::MoveHead::MoveHead(std::string name, double angle, bool is_pan_arg) 
: ActionNode::ActionNode(name), ang_value(angle), is_pan(is_pan_arg)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&MoveHead::WaitForTick, this);
}

BT::MoveHead::~MoveHead() {}

void BT::MoveHead::WaitForTick()
{
    while (ros::ok())
    {
        // ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_searchball");
        tick_engine.Wait();
        // ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_searchball");
        
        while (get_status() == BT::IDLE)
        {
            ang_value *= 0.0174533;  // DegToRad -> pi/180
    
            // set_status(BT::RUNNING);
            if (getModule("r_knee") != "direct_control_module")
            {
                setModule("direct_control_module");
                ros::Duration(1).sleep();
                ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
            }
            write_msg_x_.header.stamp = ros::Time::now();
            write_msg_y_.header.stamp = ros::Time::now();
            if (is_pan){
                if (ang_value >= 1.22173) ang_value = 1.22173;            //70 deg
                else if (ang_value <= -1.22173) ang_value = -1.22173;     //-70 deg
                write_msg_x_.name.push_back("head_pan");
                write_msg_x_.position.push_back(ang_value);
                write_joint_pub_.publish(write_msg_x_);
                ROS_COLORED_LOG("Yaw Head: %f", YELLOW, false, ang_value);
            }else{
                if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
                else if (ang_value <= -1.22173) ang_value = -1.22173;   //-70 deg
                write_msg_y_.name.push_back("head_tilt");
                write_msg_y_.position.push_back(ang_value);
                write_joint_pub_.publish(write_msg_y_);
                ROS_COLORED_LOG("Pitch Head: %f", YELLOW, false, ang_value);

            }
            
            
            ROS_SUCCESS_LOG("Searching Ball!");
            ros::Duration(2).sleep();
            set_status(BT::SUCCESS);
        }
    }
}

void BT::MoveHead::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SearchBall HALTED: Stopped search ball", "ORANGE", false, "Halted_searchball");
}
