/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef TURN_LEFT_ACTION_H
#define TURN_LEFT_ACTION_H

#include <movement_pkg/utils.h>
#include <action_node.h>


namespace BT
{
class TurnLeft : public ActionNode, public virtual utils
{
    public:
        // Constructor
        explicit TurnLeft(std::string name);
        ~TurnLeft();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();
    private:
        void turn();
        
        ros::Publisher write_joint_pub_;
        sensor_msgs::JointState write_msg_;
        std::vector<std::vector<float>> positions = loadPositions();

        double crouch_angle_ = 0.08726;  // 5°
        const int rows_ = 40;
        int turns_num_;
};
}  // namespace BT

#endif  // TURN_LEFT_ACTION_H
