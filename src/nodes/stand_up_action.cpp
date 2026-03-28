/*
    Authors:
        Pedro Deniz
        Marlene Cobian
        Iván Delgado
*/

#include "movement_pkg/nodes/stand_up_action.h"
using namespace std::chrono_literals;

BT::StandUp::StandUp(const std::string &name, const BT::NodeConfig &config) 
: BT::StatefulActionNode(name, config)
{
    //node_ = rclcpp::Node::make_shared("stand_up_action");

    if(!config.blackboard->get("node",node_)){
        throw BT::RuntimeError("StandUp: missing [node] in blackboard");
    }
    utils_ = std::make_shared<utils>(node_);
    walking_controller_ = std::make_shared<WalkingController>(node_);
    RCLCPP_INFO(node_->get_logger(), "StandUpAction constructed");
}


BT::NodeStatus BT::StandUp::onStart()
{

    if(!walking_controller_){
        RCLCPP_ERROR(node_->get_logger(),"walking_controller_ is null");
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[StandUp] started");

    action_sent_ = false; //reinicia la flag de acción
    start_time_ = std::chrono::system_clock::now(); //recupera el tiempo actual de ejecución

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BT::StandUp::onRunning(){
    if (!action_sent_){
        walking_controller_->stopWalking();
        RCLCPP_INFO(node_->get_logger(),"[StandUp] Executing STAND UP");
        action_sent_=true; //activar bandera de acción

        return BT::NodeStatus::RUNNING;
    }
    auto elapsed = std::chrono::system_clock::now() - start_time_; //Tiempo transcurrido de ejecución

    if (elapsed>5s){
        RCLCPP_INFO(node_->get_logger(),"[StandUP] DONE");
        return BT::NodeStatus::SUCCESS;

    }
    if(error_d){
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::PortsList BT::StandUp::providedPorts()
{
    return{};
}


void BT::StandUp::onHalted()
{
    RCLCPP_INFO(node_->get_logger(), "[Stand Up] HALTED: Stopped Standing Up");
    error_d=true;
}