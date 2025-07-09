/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef RIGHT_MOVE_CONDITION_H
#define RIGHT_MOVE_CONDITION_H

#include "condition_node.h"


namespace BT
{
class RightMoveCondition : public ConditionNode 
{
    public:
        explicit RightMoveCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        //button
        // Callback para el suscriptor de estado
        void robotStateCallback(const std_msgs::Int32::ConstPtr& msg);
        int current_robot_state_ = -1; 
        ros::Subscriber robot_state_sub_;
        ros::NodeHandle nh_;  
};
}  // namesapce BT

#endif  // RIGHT_MOVE_CONDITION_H
