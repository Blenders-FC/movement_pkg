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
    void halt() override;

private:
    double duration_sec_;
    std::optional<std::chrono::steady_clock::time_point> start_time_;
};
}  // namesapce BT

#endif // TIMER_CONDITION_H
