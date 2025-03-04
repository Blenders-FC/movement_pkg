/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/walk_to_target_action.h"


BT::WalkToTarget::WalkToTarget(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToTarget::WaitForTick, this);
}

BT::WalkToTarget::~WalkToTarget() {}

void BT::WalkToTarget::WaitForTick()
{
    while (true)
    {
        DEBUG_STDOUT(get_name() << "WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << "TICK RECEIVED");

        set_status(BT::RUNNING);

        // Flow for searching ball - specially when distance to ball >= 1m

        if (search_n_walk)
        {
            walkTowardsBall(0.0,-0.1);
            ros::Duration(1.0).sleep();  //wait for module DO NOT REMOVE!!!!
        }
    
        if (head_direction && angle_mov_x <= 70)
        {
            angle_mov_x += 1;
            writeHeadJoint(angle_mov_x, true);
            if (angle_mov_x == 70) head_direction = false;
        }
        else if (!head_direction && angle_mov_x >= -70)
        {
            angle_mov_x -= 1;
            writeHeadJoint(angle_mov_x, true);
            if (angle_mov_x == -70)
            {
                head_direction = true;
                turn_cnt += 1;
                if (turn_cnt == 1)
                {
                    angle_mov_y = -50;
                    ros::Duration(1.0).sleep();
                    writeHeadJoint(angle_mov_y, false);
                    ros::Duration(1.0).sleep();
                }
                else if (turn_cnt == 2) 
                {
                    turn_cnt = 0;
                    turn2search(9);
                }
            }
        }
        





        head_pan_angle_ = getHeadPan();
        head_tilt_angle_ = getHeadTilt();

        this->setModule("walking_module");
        DEBUG_STDOUT(get_name() << "Walking towards target...");
        walkTowardsTarget(head_pan_angle_, head_tilt_angle_);

        if (walkingSucced)
        {
            DEBUG_STDOUT(get_name() << "Walk to target SUCCESS");
            set_status(BT::SUCCESS);
        }
    }
}

void writeHeadJoint(double ang_value, bool is_pan)
{
    setModule("direct_control_module");
    sensor_msgs::JointState write_msg;
    write_msg.header.stamp = ros::Time::now();
        
    ang_value = (ang_value*M_PI)/180;
  
    if (is_pan){
      if (ang_value >= 1.2217) ang_value = 1.2217;            //70 deg
      else if (ang_value <= -1.2217) ang_value = -1.2217;     //-70 deg
      write_msg.name.push_back("head_pan");
      write_msg.position.push_back(ang_value);
    }else{
      if (ang_value >= 0.34906) ang_value = 0.34906;        //20 deg
      else if (ang_value <= -1.2217) ang_value = -1.2217;   //-70 deg
      write_msg.name.push_back("head_tilt");
      write_msg.position.push_back(ang_value);
    }
    write_joint_pub.publish(write_msg);
}

void BT::WalkToTarget::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    DEBUG_STDOUT("WalkToTarget HALTED: Stopped walking.");
}
