/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TURN_RIGHT_ACTION_H
#define TURN_RIGHT_ACTION_H

#include <movement_pkg/utils.h>
#include <action_node.h>


namespace BT
{
class TurnRight : public ActionNode, public virtual utils
{
    public:
        // Constructor
        explicit TurnRight(std::string name);
        ~TurnRight();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
    private:
        void turn();
        
        ros::Publisher write_joint_pub_;
        sensor_msgs::JointState write_msg_;
        double crouch_angle_ = 0.08726;  // 5°
        std::vector<std::vector<float>> positions = loadPositions();;
        const int rows_ = 40;
};
}  // namespace BT

#endif  // TURN_RIGHT_ACTION_H
