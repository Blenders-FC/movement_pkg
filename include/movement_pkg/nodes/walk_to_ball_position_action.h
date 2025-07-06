/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_TO_BALL_POSITION_ACTION_H
#define WALKING_TO_BALL_POSITION_ACTION_H

#include "movement_pkg/walking_controller.h"
#include <action_node.h>


namespace BT
{
class WalkToBallPosition : public ActionNode, public WalkingController
{
    public:
        // Constructor
        explicit WalkToBallPosition(std::string name);
        ~WalkToBallPosition();

        // The method that is going to be executed by the thread
        void WaitForTick();

        // The method used to interrupt the execution of the node
        void Halt();

    private:

        bool publishLegPlan(const std::vector<humanoid_nav_msgs::StepTarget>& walking_plan, 
                            ros::Publisher& pub)

        // Variables
        ros::Publisher leg_plan_pub_;
        
        // meters and radians
        double start_x = 0;
        double start_y = 0;
        double start_theta = 0;
        double goal_x = 1.0;
        double goal_y = 0;
        double goal_theta = 0;
};
}  // namespace BT

#endif  // WALKING_TO_BALL_POSITION_ACTION_H
