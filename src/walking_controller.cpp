/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/walking_controller.h>


WalkingController::WalkingController() : utils()
{
    // Publishers
    walk_command_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/walking/command", 10);
    set_walking_param_pub_ = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/set_params",0);
    online_step_pub_ = nh.advertise<op3_online_walking_module_msgs::Step2DArray>("/robotis_" + std::to_string(robot_id) + "/online_walking/footsteps_2d", 1);
    

    // Services
    get_param_client_ = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/get_params");
    footstep_planner_client_ = nh.serviceClient<humanoid_nav_msgs::PlanFootsteps>("/plan_footsteps");
}

WalkingController::~WalkingController() {}
  
void WalkingController::goWalk(std::string& command)
{
    this->setModule("walking_module");
    if (command == "start") 
    {
        setWalkingParam(IN_PLACE_FB_STEP_, 0, 0, true);
    }

    std_msgs::String command_msg;
    command_msg.data = command;
    walk_command_pub.publish(command_msg);
}

void WalkingController::calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle)
{
    double next_movement = current_x_move_;
    if (target_distance < 0)
        target_distance = 0.0;

    double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP_);
    accum_period_time_ += delta_time;
    if (accum_period_time_ > (current_period_time_  / 4)) 
    {
        accum_period_time_ = 0.0;
        if ((target_distance * 0.1 / 2) < current_x_move_)
            next_movement -= UNIT_FB_STEP_;
        else
            next_movement += UNIT_FB_STEP_;
    }
    fb_goal = fmin(next_movement, fb_goal);
    fb_move = fmax(fb_goal, MIN_FB_STEP_);

    double rl_goal = 0.0;
    if (fabs(target_angle) * 180 / M_PI > 5.0) 
    {
        double rl_offset = fabs(target_angle) * 0.2;
        rl_goal = fmin(rl_offset, MAX_RL_TURN_);
        rl_goal = fmax(rl_goal, MIN_RL_TURN_);
        rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN_, rl_goal);

        if (target_angle < 0)
            rl_angle *= (-1);
    }
}

void WalkingController::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
    getWalkingParam();

    current_walking_param_.balance_enable = balance;
    current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET_;
    current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET_;
    current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET_;

    set_walking_param_pub_.publish(current_walking_param_);

    current_x_move_ = x_move;
    current_r_angle_ = rotation_angle;
}

void WalkingController::getWalkingParam()
{
    op3_walking_module_msgs::GetWalkingParam walking_param_msg;
    if (get_param_client_.call(walking_param_msg))
    {
        current_walking_param_ = walking_param_msg.response.parameters;
  
        // update ui
        ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");
        return;
    } else {
        ROS_ERROR_LOG("Fail to get walking parameters", false);
        return;
    }
}

void WalkingController::stopWalking()
{
    goWalk(stop_walking_command_);
}

void WalkingController::walkToPose(double x_goal, double y_goal, double theta_goal)
{
    std::vector<op3_online_walking_module_msgs::Step2D> steps;

    bool success = callFootstepPlanner(x_goal, y_goal, theta_goal, steps);
    if (success) 
    {
        ROS_COLORED_LOG("Footstep plan successful. Sending footsteps to walking module...", CYAN, false);
        publishFootsteps(steps, 0.7); // 0.7s per step — tune as needed
    } 
    else 
    {
        ROS_ERROR_LOG("Footstep planning failed");
    }
}

bool WalkingController::callFootstepPlanner(double x_goal, double y_goal, double theta_goal, std::vector<op3_online_walking_module_msgs::Step2D>& step_list)
{
    humanoid_nav_msgs::PlanFootsteps srv;

    // Set start and goal in robot-local frame
    srv.request.start.x = 0.0;
    srv.request.start.y = 0.0;
    srv.request.start.theta = 0.0;

    srv.request.goal.x = x_goal;
    srv.request.goal.y = y_goal;
    srv.request.goal.theta = theta_goal;

    if (!footstep_planner_client_.call(srv))
    {
        ROS_ERROR_LOG("Failed to call /plan_footsteps service");
        return false;
    }

    if (!srv.response.result)
    {
        ROS_ERROR_LOG("Footstep planner returned 'false' for result");
        return false;
    }

    for (const auto& step : srv.response.footsteps)
    {
        op3_online_walking_module_msgs::Step2D step_msg;
        step_msg.step2d.x = step.pose.x;
        step_msg.step2d.y = step.pose.y;
        step_msg.step2d.theta = step.pose.theta;

        // Convert leg type from planner to OP3 walking module
        if (step.leg == humanoid_nav_msgs::StepTarget::left)
            step_msg.moving_foot = 0;  // LEFT_FOOT_SWING

        else if (step.leg == humanoid_nav_msgs::StepTarget::right)
            step_msg.moving_foot = 1;  // RIGHT_FOOT_SWING

        else
            continue;  // skip unknown leg

        step_list.push_back(step_msg);
    }

    return true;
}

void WalkingController::publishFootsteps(const std::vector<op3_online_walking_module_msgs::Step2D>& steps, double step_time)
{
    op3_online_walking_module_msgs::Step2DArray msg;
    msg.step_time = step_time;

    for (const auto& s : steps)
        msg.footsteps_2d.push_back(s);

    step_pub_.publish(msg);
}
