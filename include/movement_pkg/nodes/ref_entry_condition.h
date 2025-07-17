/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef REF_ENTRY_CONDITION
#define REF_ENTRY_CONDITION

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class RefEntryCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit RefEntryCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
};
}  // namesapce BT

#endif // RefEntryCondition
