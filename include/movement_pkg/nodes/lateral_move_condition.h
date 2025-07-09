/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef LATERAL_MOVE_CONDITION_H
#define LATERAL_MOVE_CONDITION_H

#include "condition_node.h"


namespace BT
{
class LateralMoveCondition : public ConditionNode 
{
    public:
        explicit LateralMoveCondition(const std::string &name);  // Constructor

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

#endif  // LATERAL_MOVE_CONDITION_H
