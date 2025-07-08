/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/simple_walk_action.h" 


BT::SimpleWalk::SimpleWalk(std::string name)
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&SimpleWalk::WaitForTick, this);
    // Inicializar el suscriptor de estado
    robot_state_sub_ = nh_.subscribe("/via_libre_state", 1, &SimpleWalk::robotStateCallback, this);
    ROS_INFO("SimpleWalk node initialized and subscribed to /via_libre_state");
}

BT::SimpleWalk::~SimpleWalk() {}

void BT::SimpleWalk::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_simple_walk");
        tick_engine.Wait(); // Espera a que el BT le dé un tick
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_simple_walk");

     
        while (get_status() == BT::IDLE)
        {
          
            if (current_robot_state_ >= 1 && current_robot_state_ <= 3) // Si el estado es 1, 2 o 3
            {
                ROS_INFO("Robot state is %d: Initiating simple walk (forward)", current_robot_state_);
                ROS_TAGGED_ONCE_LOG("Walking...", "TEAL", true, "Start_simple_walk");


                walking_command_ = "start";
                goWalk(walking_command_); 

                set_status(BT::RUNNING); 

                ROS_SUCCESS_LOG("Simple walk has been commanded to start.");
                // set_status(BT::SUCCESS);

            }
            else 
            {
                //ROS_WARN_LOG("SimpleWalk received a tick but current_robot_state_ is %d. Not initiating walk.", false, current_robot_state_);
                ROS_COLORED_LOG("SimpleWalk received a tick but current_robot_state_ is %d. Not initiating walk.", YELLOW, false, current_robot_state_);

                set_status(BT::IDLE);
                break; 
            }
        }
        
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE); 
}

// Callback para recibir el estado del robot
void BT::SimpleWalk::robotStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    current_robot_state_ = msg->data;
    ROS_INFO("SimpleWalk: Received robot state: %d", current_robot_state_);
}

void BT::SimpleWalk::Halt()
{
    stopWalking(); // Llama a la función de WalkingController para detener el movimiento

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("SimpleWalk HALTED: Stopped simple walking", "ORANGE", false, "Halted_simple_walking");
}