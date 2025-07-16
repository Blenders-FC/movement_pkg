/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef GOAL_DETECTED_CONDITION_H
#define GOAL_DETECTED_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class GoalsDetectedCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit GoalsDetectedCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        blenders_msgs::PointArray goal_posts_;
};
}  // namesapce BT

#endif // GOAL_DETECTED_CONDITION_H
