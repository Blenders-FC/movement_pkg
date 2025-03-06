/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TURN_LEFT_ACTION_H
#define TURN_LEFT_ACTION_H

#include <movemente_pkg/nodes/utils.h>
#include <action_node.h>

namespace BT
{
class LeftRight : public ActionNode, public utils
{
    public:
        // Constructor
        explicit LeftRight(std::string name);
        ~LeftRight();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
    private:
        void turn();
        
        ros::Publisher write_joint_pub_;
        sensor_msgs::JointState write_msg_;
        double rest_inc_giro_ = 0.08726;  // 5°
        std::vector<std::vector<float>> positions = loadPositions();;
        const int rows_ = 40;
};
}  // namespace BT

#endif  // TURN_LEFT_ACTION_H
