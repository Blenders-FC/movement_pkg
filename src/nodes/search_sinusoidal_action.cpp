/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/search_sinusoidal_action.h"


BT::SearchBallSinusoidal::SearchBallSinusoidal(std::string name) 
: ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;
    write_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/robotis_" + std::to_string(robot_id) + "/direct_control/set_joint_states", 0);
    thread_ = std::thread(&SearchBallSinusoidal::WaitForTick, this);
}

BT::SearchBallSinusoidal::~SearchBallSinusoidal() {}

void BT::SearchBallSinusoidal::WaitForTick()
{
    while (ros::ok())
    {
        // ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_searchball_sinusoidal");
        tick_engine.Wait();
        // ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_searchball_sinusoidal");
        
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

            // Flow for searching ball - with sinusoidal function
            t_ += 0.1;

            
            x_target_ = 60*sin(t_);
            y_target_ = 15*cos(t_*5) - 30;

            rightLeft = turn_cnt_%2;

            ROS_COLORED_LOG("x target: %f", YELLOW, true, x_target_);
            ROS_COLORED_LOG("y target: %f", YELLOW, true, y_target_);

            dx = x_target_ - x_target_past_;
            x_target_past_= x_target_;
            // if ((turn_cnt_ == 0) && (dx <= 0)){
            //     turn_cnt_ += 1;
            // }

            //
            if (((!rightLeft) && (dx <= 0)) || ((rightLeft) && (dx >= 0))){
                turn_cnt_++;
            }
            std::cout << "cont:" << turn_cnt_ << std::endl;
            // if ((turn_cnt_ >= 2 )&& (!headDown)) {
            //     headDown = 1;
            //     y_target_ = -40;
            //     writeHeadJoint(y_target_, false);

            //     //angle_mov_x_ 2;
            //     ROS_COLORED_LOG("New tilt angle position: %f", CYAN, false, y_target_);
                
            //     ros::Duration(1.0).sleep();

            // } 
            if (turn_cnt_ >= 2) {
                turn_cnt_ = 0;
                ROS_COLORED_LOG("Couldn't find ball! Changing the search position...", YELLOW, false);
                set_status(BT::FAILURE);
                head_direction_ = true;
                break;
            }

            writeHeadJoint(x_target_, y_target_);
            ros::Duration(0.5).sleep();
            
           // ROS_SUCCESS_LOG("Searching Ball!");
            set_status(BT::SUCCESS);
        }
    }
}

void BT::SearchBallSinusoidal::writeHeadJoint(double ang_valueX, double ang_valueY)
{
    write_msg.header.stamp = ros::Time::now();        
    ang_valueX *= 0.0174533;  // DegToRad -> pi/180
    ang_valueY *= 0.0174533;  // DegToRad -> pi/180
  
    
      if (ang_valueX >= 1.22173) ang_valueX = 1.22173;            //70 deg
      else if (ang_valueX <= -1.22173) ang_valueX = -1.22173;     //-70 deg
      write_msg.name.push_back("head_pan");
      write_msg.position.push_back(ang_valueX);    

      if (ang_valueY >= 0.34906) ang_valueY= 0.34906;        //20 deg
      else if (ang_valueY <= -1.22173) ang_valueY = -1.22173;   //-70 deg
      write_msg.name.push_back("head_tilt");
      write_msg.position.push_back(ang_valueY);
      write_joint_pub_.publish(write_msg);
    
}

void BT::SearchBallSinusoidal::Halt()
{
    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SearchBall HALTED: Stopped search ball", "ORANGE", false, "Halted_searchball");
}
