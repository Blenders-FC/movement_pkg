#include "movement_pkg/nodes/repeat_n_times.h"


BT::RepeatNTimes::RepeatNTimes(const std::string &name) 
: BT::ConditionNode(name), utils(){}

BT::ReturnStatus BT::RepeatNTimes::Tick()
{
    // Condition checking and state update
    while (ros::ok())
    {
        m_cnt.turncnt = cnt_init;
        if (m_cnt.turncnt < m_turns) 
        {
            ROS_COLORED_LOG("turns: %d are LESSER than threshold ", CYAN, true, m_cnt.turncnt);
            cnt_init += 1;
            blackboard.setTarget("m_turncnt",m_cnt);
            set_status(BT::SUCCESS);
            return BT::SUCCESS;
        }
        else
        {
                ROS_COLORED_LOG("turns %d are higher than threshold", CYAN, false, m_cnt.turncnt);
                m_turncnt = blackboard.getTarget("m_turncnt");
                cnt_init = m_turncnt->turncnt;
                //m_turncnt = 0;
                set_status(BT::FAILURE);
                return BT::FAILURE;
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    return BT::FAILURE;
}
