/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef CAN_MOVE_CONDITION_H
#define CAN_MOVE_CONDITION_H

#include "condition_node.h"


namespace BT
{
class CanMoveCondition : public ConditionNode 
{
    public:
        explicit CanMoveCondition(const std::string &name);  // Constructor

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

#endif  // CAN_MOVE_CONDITION_H
