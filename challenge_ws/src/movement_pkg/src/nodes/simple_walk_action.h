/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef SIMPLE_WALK_ACTION_H
#define SIMPLE_WALK_ACTION_H

#include "movement_pkg/walking_controller.h" 
#include <action_node.h>                     
#include <std_msgs/Int32.h>                  

namespace BT
{
class SimpleWalk : public ActionNode, public WalkingController
{
    public:
        explicit SimpleWalk(std::string name);  
        ~SimpleWalk();                          

       
        void WaitForTick();

        void Halt();

    private:
        std::string walking_command_; 

    
        int current_robot_state_ = -1; 
                                       

        ros::Subscriber robot_state_sub_; 
        ros::NodeHandle nh_;              

        
        
        void robotStateCallback(const std_msgs::Int32::ConstPtr& msg);
};
}  

#endif  // SIMPLE_WALK_ACTION_H