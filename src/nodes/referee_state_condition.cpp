
/*
    Authors:
        Pedro Deniz
        Marlene Cobian

    Iván Delgado
*/

#include "movement_pkg/nodes/referee_state_condition.h"


BT::RefereeStateCondition::RefereeStateCondition(const std::string &name, const BT::NodeConfig &config) 
: BT::ConditionNode(name, config) {
    node_ = rclcpp::Node::make_shared("referee_state_condition");
}
BT::PortsList BT::RefereeStateCondition::providedPorts()
{
    return {};  // este nodo no usa puertos de entrada/salida
}
BT::NodeStatus BT::RefereeStateCondition::tick()
{
    
    while (rclcpp::ok())
    {
            // Condition checking and state update
        int refereeState = cb_data_manager_.getRefereeState();   

        if (refereeState == referee::STILL || refereeState == referee::GET_FAR)
        {   
            RCLCPP_INFO(rclcpp::get_logger("ReferreStateCondition"), "Not allowed to play by referee");
            return BT::NodeStatus::FAILURE;

        }
        else
        {   
            RCLCPP_INFO(rclcpp::get_logger("ReferreStateCondition"), "Play");
            RCLCPP_INFO(rclcpp::get_logger("ReferreStateCondition"), "referee state  is %d", refereeState);
            return BT::NodeStatus::SUCCESS;
        }
    }

    RCLCPP_ERROR(rclcpp::get_logger("ReferreStateCondition"), "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE;
}