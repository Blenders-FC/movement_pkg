/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef BALL_DETECTED_CONDITION_H
#define BALL_DETECTED_CONDITION_H

#include "movement_pkg/cb_data_manager.h"
#include "condition_node.h"


namespace BT
{
class BallDetectedViolaJonesCondition : public ConditionNode, public CBDataManager
{
    public:
        explicit BallDetectedViolaJonesCondition(const std::string &name);  // Constructor

        // Behavior Tree Tick function
        BT::ReturnStatus Tick() override;

    private:
        geometry_msgs::Point ball_center_position_;
        bool ball_detected;
        int no_detection_counter = 0;
        const int max_no_detection_count = 10;   // number of frames allowed before setting a failure
};
}  // namesapce BT

#endif // BALL_DETECTED_CONDITION_H
