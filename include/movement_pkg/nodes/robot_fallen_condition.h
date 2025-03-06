/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef ROBOT_FALLEN_CONDITION_H
#define ROBOT_FALLEN_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class RobotFallenCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit RobotFallenCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        double pitch;
        double alpha = 0.4;
        double present_pitch_;
        const double FALL_FORWARD_LIMIT = 55;
        const double FALL_BACKWARDS_LIMIT = -55;
        
        
        bool DEBUG_PRINT_ = false;
};
}  // namesapce BT

#endif  // ROBOT_FALLEN_CONDITION_H
