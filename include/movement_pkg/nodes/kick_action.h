/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef KICK_ACTION_H
#define KICK_ACTION_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <movemente_pkg/nodes/utils.h>

namespace BT
{
class KickAction : public ActionNode
{
public:
    // Constructor
    explicit KickAction(std::string name);
    ~KickAction();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();
private:
    ros::Publisher action_pose_pub_;

    void goAction(int page);
};
}  // namespace BT

#endif  // KICK_ACTION_H
