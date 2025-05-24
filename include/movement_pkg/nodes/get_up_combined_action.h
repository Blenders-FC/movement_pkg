/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef GET_UP_COMBINED_ACTION_H
#define GET_UP_COMBINED_ACTION_H

#include <action_node.h>
#include "movement_pkg/cb_data_manager.h"

namespace BT
{
class GetUpCombined : public ActionNode, public CBDataManager
{
public:
    // Constructor
    explicit GetUpCombined(std::string name);
    ~GetUpCombined();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
private:
    double pitch;
    double alpha = 0.4;
    double present_pitch_ = 0;
    const double FALL_FORWARD_LIMIT = 55;
    const double FALL_BACKWARDS_LIMIT = -55;
};
}  // namespace BT

#endif  // GET_UP_COMBINED_ACTION_H
