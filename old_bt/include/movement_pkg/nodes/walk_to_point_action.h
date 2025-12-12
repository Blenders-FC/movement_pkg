/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_POINT_ACTION_H
#define WALKING_TO_POINT_ACTION_H

#include "movement_pkg/walking_controller.h"
#include <action_node.h>


namespace BT
{
class WalkToPoint : public ActionNode, public WalkingController
{
    public:
        // Constructor
        explicit WalkToPoint(std::string name);
        ~WalkToPoint();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:
        //  Auxiliar methods
        void walkTowardsPoint();

        // Variables
        double distance_to_ball = 1.0;  // Default value
        double pan_angle_to_ball = 0.0;  // Default value
        double walked_distance = 0.0;
        double accum_rotation = 0.0;
        double prev_delta_angle = 0.0;
        double fb_move;
        double rl_angle;
        double distance_to_walk;
        const double distance_to_kick_ = 0.30;  // 0.22
        const double CAMERA_HEIGHT_ = 0.46;
        const double hip_pitch_offset_ = 0.12217305; //7°
        double current_x_move_ = 0.0125;
        bool walkingSucced = false;
        std_msgs::String walk_command;
        ros::Time prev_time_walk_ = ros::Time::now();

        // Blackboard
        const TargetInfo* ball = blackboard.getTarget("ball");
};
}  // namespace BT

#endif  // WALKING_TO_POINT_ACTION_H
