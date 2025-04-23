/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/nodes/long_kick_action.h>

BT::LongKick::LongKick(std::string name) : ActionNode::ActionNode(name), utils()
{
    // Publisher
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/set_joint_states", 0);

    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&LongKick::WaitForTick, this);
}

BT::LongKick::~LongKick() {}

void BT::LongKick::WaitForTick()
{
    while(ros::ok())
    {
        // Waiting for the first tick to come
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // Running state
            set_status(BT::RUNNING);

            ROS_TAGGED_ONCE_LOG("Long kicking!");
            //node loop
            write_msg_.header.stamp = ros::Time::now();
            
            if (getModule("r_knee") != "none")
            {
                setModule("none");
                ros::Duration(1).sleep();
            }

            ros::Duration(0.1).sleep();
            kick();
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

void BT::LongKick::kick()
{    
    //Detenerse
    write_msg_.name.push_back("r_ank_pitch");
    write_msg_.position.push_back(positions[rows_-1][0]);
    write_msg_.name.push_back("r_knee");
    write_msg_.position.push_back(positions[rows_-1][1]);
    write_msg_.name.push_back("r_hip_pitch");
    write_msg_.position.push_back(positions[rows_-1][2] + crouch_angle_);
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(positions[rows_-1][3]);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(positions[rows_-1][4]);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(positions[rows_-1][5] - crouch_angle_);
    write_joint_pub_.publish(write_msg_);
     
    //Inclinarse para alinear el centro de masa
    ros::Duration(1).sleep();
    write_msg_.name.push_back("l_hip_roll");
    write_msg_.position.push_back(0.17);
    write_msg_.name.push_back("r_hip_roll");
    write_msg_.position.push_back(0.17);
    write_msg_.name.push_back("r_ank_roll");
    write_msg_.position.push_back(-0.15);
    write_msg_.name.push_back("l_ank_roll");
    write_msg_.position.push_back(-0.45);
    write_joint_pub_.publish(write_msg_);
     
    //Posicion de seguridad
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(0.7091);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(1.5287);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(-0.9474 - crouch_angle_);
    write_msg_.name.push_back("l_ank_roll");
    write_msg_.position.push_back(0);
    write_joint_pub_.publish(write_msg_);
     
    //Patada
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(0.0046);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(0.7420);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(-1.2287 - crouch_angle_);
    write_joint_pub_.publish(write_msg_);

    ros::Duration(0.05).sleep();
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(0);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(-1.5 - crouch_angle_);
    write_joint_pub_.publish(write_msg_);
     
    //Posiciòn de seguridad
    ros::Duration(0.1).sleep();
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(0.7091);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(1.8);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(-1.4 - crouch_angle_);
    write_msg_.name.push_back("l_ank_roll");
    write_msg_.position.push_back(0);
    write_joint_pub_.publish(write_msg_);
     
    //Regreso
    ros::Duration(0.2).sleep();
    write_msg_.name.push_back("l_hip_roll");
    write_msg_.position.push_back(-0.0873);
    write_msg_.name.push_back("r_hip_roll");
    write_msg_.position.push_back(0.0873);
    write_msg_.name.push_back("l_ank_roll");
    write_msg_.position.push_back(-0.0873);
    write_msg_.name.push_back("r_ank_roll");
    write_msg_.position.push_back(0.0873);
    
    write_msg_.name.push_back("l_ank_pitch");
    write_msg_.position.push_back(positions[rows_-1][3]);
    write_msg_.name.push_back("l_knee");
    write_msg_.position.push_back(positions[rows_-1][4]);
    write_msg_.name.push_back("l_hip_pitch");
    write_msg_.position.push_back(positions[rows_-1][5] - crouch_angle_);
    write_joint_pub_.publish(write_msg_);
}

void BT::LongKick::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("LongKick HALTED: Stopped long kick");
}