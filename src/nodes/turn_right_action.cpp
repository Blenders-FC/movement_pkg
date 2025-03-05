/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movemente_pkg/nodes/turn_right_action.h>

BT::TurnRight::TurnRight(std::string name) : ActionNode::ActionNode(name), utils()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&TurnRight::WaitForTick, this);
}

BT::TurnRight::~TurnRight() {}

void BT::TurnRight::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        // Perform action...
        if (get_status() != BT::HALTED)
        {
            //node loop
            sensor_msgs::JointState write_msg;
            write_msg.header.stamp = ros::Time::now();
            
            setModule("none");
            ros::Duration(1).sleep();
                ros::Duration(0.1).sleep();
    
        }
    }
}

void BT::TurnRight::turn()
{    
    // Getting right foot up
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(-0.7091);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(-1.4131);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(0.7091 + rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0.1746*1.5);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(-0.1746*1.5);

    write_msg.name.push_back("l_hip_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_hip_roll");
    write_msg.position.push_back(0.0873);
    write_msg.name.push_back("l_ank_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_ank_roll");
    write_msg.position.push_back(0.0873);
    write_joint_pub2.publish(write_msg);

    // Getting right foot down
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(positions[rows-1][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(positions[rows-1][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(positions[rows-1][2] + rest_inc_giro);
    write_joint_pub2.publish(write_msg);

    // Getting left foot up
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(0.7091);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(1.4131);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(-0.7091 - rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(0);
    write_joint_pub2.publish(write_msg);

    // Getting left foot down
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(positions[rows-1][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(positions[rows-1][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(positions[rows-1][5] - rest_inc_giro);
    write_joint_pub2.publish(write_msg);
}

void BT::TurnRight::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("TurnRight HALTED: Stopped walking.");
}