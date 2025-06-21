#ifndef REPEAT_N_TIMES_H
#define REPEAT_N_TIMES_H

#include "condition_node.h"
#include <string>
#include "movement_pkg/utils.h"
namespace BT
{
class RepeatNTimes : public ConditionNode, public virtual utils
{
    public:
        explicit RepeatNTimes(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        bool already_logged_ = false;
        //std::pair<std::string, std::string> robot_status_;
        int m_turncnt=0;
        int m_turns=2; 

};
}  // namesapce BT



#endif // REPEAT_N_TIMES_H