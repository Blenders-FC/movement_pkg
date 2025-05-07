/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/nodes/turn_left_action.h>

BT::TurnLeft::TurnLeft(std::string name)
: ActionNode::ActionNode(name), utils(), turns_num_(turns)
{
    // Publisher
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/set_joint_states", 0);

    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&TurnLeft::WaitForTick, this);
}

BT::TurnLeft::~TurnLeft() {}

void BT::TurnLeft::WaitForTick()
{
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_turn_l");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_turn_l");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            ROS_TAGGED_ONCE_LOG("Turning left in place!", "CYAN", false, "Turn_l");

            // Running state
            set_status(BT::RUNNING);

            //node loop
            write_msg_.header.stamp = ros::Time::now();
            
            if (getModule("l_knee") != "none")
            {
                setModule("none");
                ros::Duration(1).sleep();
            }

            for (int i = 0; i < turns_num_; i++)
            {
                turn();
            }

            ROS_SUCCESS_LOG("Turning left has finished successfully!");
            set_status(BT::SUCCESS);
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::TurnLeft::turn()
{    
    // Getting left foot up
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(0.7091);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(1.4131);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(-0.7091 - crouch_angle_);
    write_msg_.name.push_back("r_hip_yaw");
    write_msg_.position.push_back(0.1746*1.5);
    write_msg_.name.push_back("l_hip_yaw");
    write_msg_.position.push_back(-0.1746*1.5);
    write_joint_pub_.publish(write_msg_);
    
    write_msg_.name.push_back("l_hip_roll");
    write_msg_.position.push_back(-0.0873);
    write_msg_.name.push_back("r_hip_roll");
    write_msg_.position.push_back(0.0873);
    write_msg_.name.push_back("l_ank_roll");
    write_msg_.position.push_back(-0.0873);
    write_msg_.name.push_back("r_ank_roll");
    write_msg_.position.push_back(0.0873);
    write_joint_pub_.publish(write_msg_);
    
    // Getting left foot down
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(positions[rows_-1][3]);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(positions[rows_-1][4]);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(positions[rows_-1][5] - crouch_angle_);
    write_joint_pub_.publish(write_msg_);
    
    // Getting right foot up
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("r_ank_pitch");
    write_msg_.position.push_back(-0.7091);
    write_msg_.name.push_back("r_knee");
    write_msg_.position.push_back(-1.4131);
    write_msg_.name.push_back("r_hip_pitch");
    write_msg_.position.push_back(0.7091 + crouch_angle_);
    write_msg_.name.push_back("r_hip_yaw");
    write_msg_.position.push_back(0);
    write_msg_.name.push_back("l_hip_yaw");
    write_msg_.position.push_back(0);

    // Getting right foot down
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("r_ank_pitch");
    write_msg_.position.push_back(positions[rows_-1][0]);
    write_msg_.name.push_back("r_knee");
    write_msg_.position.push_back(positions[rows_-1][1]);
    write_msg_.name.push_back("r_hip_pitch");
    write_msg_.position.push_back(positions[rows_-1][2] + crouch_angle_);
    write_joint_pub_.publish(write_msg_);
}

void BT::TurnLeft::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("TurnLeft HALTED: Stopped turning in place", "ORANGE", false, "Halted_turn_l");
}