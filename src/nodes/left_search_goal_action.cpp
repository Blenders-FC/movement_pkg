/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/left_search_goal_action.h"


BT::LeftSearchGoal::LeftSearchGoal(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&LeftSearchGoal::WaitForTick, this);
}

BT::LeftSearchGoal::~LeftSearchGoal() {}

void BT::LeftSearchGoal::WaitForTick()
{
    while (ros::ok())
    {
        // ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_searchgoal_sinusoidal");
        tick_engine.Wait();
        // ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_searchgoal_sinusoidal");
        
        while (get_status() == BT::IDLE)
        {
            //turn_cnt_ = 0;

            // set_status(BT::RUNNING);
            if (getModule("r_knee") != "direct_control_module")
            {
                setModule("direct_control_module");
                ros::Duration(1).sleep();
                ROS_COLORED_LOG("Set Module to direct_control_module", YELLOW, false);
            }

            // Flow for searching goal - with sinusoidal function
            t_ += 0.1;

            x_target_ = 60*sin(t_) + 30;  // 60*sin(t_);
            // rightLeft = turn_cnt_%2;

            ROS_COLORED_LOG("x target: %f", YELLOW, true, x_target_);

            dx = x_target_ - x_target_past_;
            x_target_past_= x_target_;
            
            // if (((!rightLeft) && (dx <= 0)) || ((rightLeft) && (dx >= 0))){
            //     turn_cnt_++;
            // }

            // if (turn_cnt_ >= 2) {
            //     turn_cnt_ = 0;
            //     ROS_COLORED_LOG("Couldn't find goal! Changing the search position...", YELLOW, false);
            //     set_status(BT::FAILURE);
            //     head_direction_ = true;
            //     break;
            // }

            writeHeadJoint(x_target_);
            ros::Duration(0.5).sleep();
            
           // ROS_SUCCESS_LOG("Searching goal!");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::LeftSearchGoal::writeHeadJoint(double ang_valueX)
{
    write_msg.header.stamp = ros::Time::now();        
    ang_valueX *= 0.0174533;  // DegToRad -> pi/180
  
    
    if (ang_valueX >= 1.22173) ang_valueX = 1.22173;            //70 deg
    else if (ang_valueX <= -1.22173) ang_valueX = -1.22173;     //-70 deg
    write_msg.name.push_back("head_pan");
    write_msg.position.push_back(ang_valueX);

    write_joint_pub_.publish(write_msg);
}

void BT::LeftSearchGoal::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SearchGoal HALTED: Stopped left search goal", "ORANGE", false, "Halted_leftsearchgoal");
}
