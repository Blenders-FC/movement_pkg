/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_TARGET_ACTION_H
#define WALKING_TO_TARGET_ACTION_H

#include "movement_pkg/walking_controller.h" 
#include "movement_pkg/cb_data_manager.h"    
#include <action_node.h>                     
#include <std_msgs/Int32.h>                  
#include <std_msgs/String.h>                 

namespace BT
{
class WalkToTarget : public ActionNode, public WalkingController, public CBDataManager
{
    public:
        // Constructor
        explicit WalkToTarget(std::string name);
        ~WalkToTarget();

      
        void WaitForTick();

     
        void Halt();

    private:
        // Callback para el suscriptor de estado
        void robotStateCallback(const std_msgs::Int32::ConstPtr& msg);

        int current_robot_state_ = -1; 
                                      
        ros::Subscriber robot_state_sub_;
        ros::NodeHandle nh_;              

 
};
}  

#endif  // WALKING_TO_TARGET_ACTION_H