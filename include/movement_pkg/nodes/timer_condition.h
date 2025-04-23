/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TIMER_CONDITION_H
#define TIMER_CONDITION_H

#include <condition_node.h>
#include <chrono>
#include <optional>
#include "movement_pkg/utils.h"


namespace BT
{
class TimerCondition : public ConditionNode, public virtual utils
{
public:
    // Constructor
    explicit TimerCondition(const std::string& name, double duration);

    // Behavior Tree Tick function
    BT::ReturnStatus Tick() override;

private:
    double duration_sec_;
    std::chrono::steady_clock::time_point _start_time;
    bool timer_started_;
};
}  // namesapce BT

#endif // TIMER_CONDITION_H
