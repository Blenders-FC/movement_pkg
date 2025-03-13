/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef START_BUTTON_CONDITION_H
#define START_BUTTON_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class StartButtonCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit StartButtonCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        //button
        bool start_button_flag_ = false;
        bool already_logged_ = false;
};
}  // namesapce BT

#endif  // START_BUTTON_CONDITION_H
