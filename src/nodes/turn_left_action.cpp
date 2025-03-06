/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movemente_pkg/nodes/turn_right_action.h>

BT::LeftRight::LeftRight(std::string name) : ActionNode::ActionNode(name), utils()
{
    // Publisher
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/set_joint_states", 0);

    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&LeftRight::WaitForTick, this);
}

BT::LeftRight::~LeftRight() {}

void BT::LeftRight::WaitForTick()
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
        while (get_status() != BT::HALTED)
        {
            DEBUG_STDOUT("Turning in place!");
            //node loop
            write_msg_.header.stamp = ros::Time::now();
            
            if (get_joint_module_client != "none")
            {
                setModule("none");
                ros::Duration(1).sleep();
            }

            ros::Duration(0.1).sleep();
            turn();
        }
    }
}

void BT::LeftRight::turn()
{    
    // Getting left foot up
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(0.7091);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(1.4131);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(-0.7091 - rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0.1746*1.5);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(-0.1746*1.5);
    write_joint_pub.publish(write_msg);
    
    write_msg.name.push_back("l_hip_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_hip_roll");
    write_msg.position.push_back(0.0873);
    write_msg.name.push_back("l_ank_roll");
    write_msg.position.push_back(-0.0873);
    write_msg.name.push_back("r_ank_roll");
    write_msg.position.push_back(0.0873);
    write_joint_pub.publish(write_msg);
    
    // Getting left foot down
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("l_ank_pitch");
    write_msg.position.push_back(positions[rows_-1][3]);
    write_msg.name.push_back("l_knee");
    write_msg.position.push_back(positions[rows_-1][4]);
    write_msg.name.push_back("l_hip_pitch");
    write_msg.position.push_back(positions[rows_-1][5] - rest_inc_giro);
    write_joint_pub.publish(write_msg);
    
    // Getting right foot up
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(-0.7091);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(-1.4131);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(0.7091 + rest_inc_giro);
    write_msg.name.push_back("r_hip_yaw");
    write_msg.position.push_back(0);
    write_msg.name.push_back("l_hip_yaw");
    write_msg.position.push_back(0);

    // Getting right foot down
    ros::Duration(0.1).sleep();
    write_msg.name.push_back("r_ank_pitch");
    write_msg.position.push_back(positions[rows_-1][0]);
    write_msg.name.push_back("r_knee");
    write_msg.position.push_back(positions[rows_-1][1]);
    write_msg.name.push_back("r_hip_pitch");
    write_msg.position.push_back(positions[rows_-1][2] + rest_inc_giro);
    write_joint_pub.publish(write_msg);
}

void BT::LeftRight::Halt()
{
    set_status(BT::HALTED);
    DEBUG_STDOUT("LeftRight HALTED: Stopped turning in place");
}