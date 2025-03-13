/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef WALKING_CONTROLLER_H
#define WALKING_CONTROLLER_H

#include "movement_pkg/utils.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"


class WalkingController : public virtual utils
{
public:
    // Constructor
    explicit WalkingController();
    ~WalkingController();

    // External methods
    void goWalk(std::string& command);
    void stopWalking();
    void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle);
    void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance);

    // Public publisher
    ros::Publisher walk_command_pub;

private:

    // Auxiliar method
    void getWalkingParam();

    // Private ros variables
    ros::ServiceClient get_param_client_;
    ros::Publisher set_walking_param_pub_;
    ros::Time prev_time_walk_;
    op3_walking_module_msgs::WalkingParam current_walking_param_;

    const double IN_PLACE_FB_STEP_ = -0.003;
    const double UNIT_FB_STEP_ = 0.002;
    const double UNIT_RL_TURN_ = 0.00872665;        //0.5°
    const double MAX_FB_STEP_ = 0.0175;             //0.04;  //0.007;  //0.015;
    const double MAX_RL_TURN_ =  0.26179939;        //15°
    const double MIN_FB_STEP_ = 0.0125;             //0.024;  //0.003;  //0.01;
    const double MIN_RL_TURN_ = 0.08726646;         //5°
    const double SPOT_FB_OFFSET_ = 0.0;
    const double SPOT_RL_OFFSET_ = 0.0;
    const double SPOT_ANGLE_OFFSET_ = 0.0;
    double accum_period_time_ = 0.0;
    double current_period_time_ = 0.6;
    double current_x_move_ = 0.005;
    double current_r_angle_ = 0.0;
    std::string stop_walking_command_ = "stop";
};

#endif  // WALKING_CONTROLLER_H
