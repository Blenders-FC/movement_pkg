/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef HEAD_TO_HOME_RESET_H
#define HEAD_TO_HOME_RESET_H

#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class HeadToHomeReset : public ActionNode, public CBDataManager
{
    public:
        // Constructor
        explicit HeadToHomeReset(std::string name);
        ~HeadToHomeReset();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:
        //  Auxiliar methods
        void writeHeadJoint(double ang_value, bool is_pan);

        // ROS variable
        ros::Publisher write_joint_pub_;
        sensor_msgs::JointState write_msg_;

        TargetInfo m_turncnt;
};
}  // namespace BT

#endif  // HEAD_TO_HOME_ACTION_H
