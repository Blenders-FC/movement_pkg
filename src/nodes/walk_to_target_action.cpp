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
    // Inicializar el suscriptor de estado
    robot_state_sub_ = nh_.subscribe("/robotis_" + std::to_string(robot_id) + "/via_libre_state", 1, &WalkToTarget::robotStateCallback, this);
    ROS_INFO("WalkToTarget node initialized and subscribed to /via_libre_state (simplified for lateral movement).");
}

BT::WalkToTarget::~WalkToTarget() {}

void BT::WalkToTarget::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_lateral_move_action");
        tick_engine.Wait(); 
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_lateral_move_action");

        set_status(BT::RUNNING); 

        
        while (ros::ok() && get_status() == BT::RUNNING)
        {
            if (current_robot_state_ == 3)
            {
                ROS_INFO_THROTTLE(1.0, "Robot state is 0: Moving 0.2 to the right (Y-axis) continuously.");
               
                setWalkingParam(0.0, -0.2, 0.0, true); // x_move=0, y_move=-0.2 (derecha), rot_angle=0, enable=true
                
                std_msgs::String command_msg;
                command_msg.data = "start";
                walk_command_pub.publish(command_msg);
                
                ros::spinOnce(); 
                ros::Rate(10).sleep(); 
            }
            else 
            {
                if (current_robot_state_ != -1) { 
                    ROS_INFO("Robot state is %d (not 0): Stopping lateral movement.", current_robot_state_);
                    set_status(BT::SUCCESS);
                    
                } else {
                    
                    ROS_WARN_THROTTLE(2.0, "WalkToTarget: Waiting for valid robot state. Current: %d. Stopping movement.", current_robot_state_);
                }
                stopWalking(); 


                set_status(BT::IDLE);
                break; 
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly. Setting status to FAILURE.", false);
    set_status(BT::FAILURE); 
}

// Callback para recibir el estado del robot desde el tópico /via_libre_state
void BT::WalkToTarget::robotStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    current_robot_state_ = msg->data;
    ROS_INFO("WalkToTarget: Received robot state: %d", current_robot_state_);
}


void BT::WalkToTarget::Halt()
{
    stopWalking(); 

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("WalkToTarget HALTED: Stopped lateral movement", "ORANGE", false, "Halted_lateral_move");
}