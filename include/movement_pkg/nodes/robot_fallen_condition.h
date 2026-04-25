/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef ROBOT_FALLEN_CONDITION_H
#define ROBOT_FALLEN_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "behaviortree_cpp/condition_node.h"


namespace BT
{
class RobotFallenCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit RobotFallenCondition(const std::string &name, const BT::NodeConfig& config);  // Constructor

        static BT::PortsList providedPorts();
        // Behavior Tree Tick function
        BT::NodeStatus tick() override;

    private:
        double pitch;
        double alpha = 0.4;
        double present_pitch_ = 0;
        const double FALL_FORWARD_LIMIT = 55;
        const double FALL_BACKWARDS_LIMIT = -55;
};
}  // namesapce BT

#endif  // ROBOT_FALLEN_CONDITION_H
