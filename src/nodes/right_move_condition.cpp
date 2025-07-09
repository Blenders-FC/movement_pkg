/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/right_move_condition.h"


BT::RightMoveCondition::RightMoveCondition(const std::string &name) 
: BT::ConditionNode(name) {
   robot_state_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/via_libre_state", 1, &RightMoveCondition::robotStateCallback, this);
}

BT::ReturnStatus BT::RightMoveCondition::Tick()
{
    // Condition checking and state update
    while(ros::ok())
    {        

        if (current_robot_state_ == 1)
        {
            ROS_SUCCESS_LOG("Robot get valid data to move right");
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
            
        }
        else
        {
            ROS_COLORED_LOG("Moving left", CYAN, false);
            set_status(BT::FAILURE);
            return BT::FAILURE;
            
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}

// Callback para recibir el estado del robot desde el tópico /via_libre_state
void BT::RightMoveCondition::robotStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    current_robot_state_ = msg->data;
    ROS_INFO("WalkToTarget: Received robot state: %d", current_robot_state_);
}
