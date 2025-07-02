/*
    Authors:
        Vicman Gil
*/

#include "movement_pkg/nodes/lower_head_action.h"


BT::LowerHead::LowerHead(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&LowerHead::WaitForTick, this);
}

BT::LowerHead::~LowerHead() {}

void BT::LowerHead::WaitForTick()
{
    while (ros::ok())
    {
        // ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_searchball");
        tick_engine.Wait();
        // ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_searchball");
        
        while (get_status() == BT::IDLE)
        {
            ang_value = -1.2;
    
            // set_status(BT::RUNNING);
            if (getModule("r_knee") != "direct_control_module")
            {
                setModule("direct_control_module");
                ros::Duration(1).sleep();
                ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
            }
            write_msg_x_.header.stamp = ros::Time::now();
            write_msg_y_.header.stamp = ros::Time::now();
            if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
            else if (ang_value <= -1.22173) ang_value = -1.22173;   //-70 deg
            write_msg_y_.name.push_back("head_tilt");
            write_msg_y_.position.push_back(ang_value);
            write_joint_pub_.publish(write_msg_y_);
            
            
            ROS_SUCCESS_LOG("Searching Ball!");
            ros::Duration(2).sleep();
            set_status(BT::SUCCESS);
        }
    }
}

void BT::LowerHead::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SearchBall HALTED: Stopped search ball", "ORANGE", false, "Halted_searchball");
}
