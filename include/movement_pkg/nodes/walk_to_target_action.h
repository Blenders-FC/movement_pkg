/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_TARGET_ACTION_H
#define WALKING_TO_TARGET_ACTION_H

#include "movement_pkg/nodes/walking_controller.h"
#include "movement_pkg/cb_data_manager.h"
#include <action_node.h>


namespace BT
{
class WalkToTarget : public ActionNode
{
public:
    // Constructor
    explicit WalkToTarget(std::string name);
    ~WalkToTarget();

    // The method that is going to be executed by the thread
    void WaitForTick();

    // The method used to interrupt the execution of the node
    void Halt();

private:
    //  Auxiliar methods
    void walkTowardsTarget(double head_pan_angle, double head_tilt_angle);
    double calculateDistance(duble head_tilt);

    // Variables
    double head_pan_angle_;
    double head_tilt_angle_;
    double fb_move;
    double rl_angle;
    double distance_to_walk;
    const double distance_to_kick_ = 0.30;  // 0.22
    const double CAMERA_HEIGHT_ = 0.46;
    const double hip_pitch_offset_ = 0.12217305; //7°
    bool walkingSucced = false;
    std_msgs::String walk_command;
};
}  // namespace BT

#endif  // WALKING_TO_TARGET_ACTION_H
