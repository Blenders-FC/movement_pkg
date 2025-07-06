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
    footstep_walk_command_pub = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/footstep_walking/command", 10);
    set_walking_param_pub_ = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/set_params",0);
    balance_enable_pub_ = nh.advertise<std_msgs::String>("/robotis_" + std::to_string(robot_id) + "/online_walking/wholebody_balance_msg", 1);
    online_step_pub_ = nh.advertise<op3_online_walking_module_msgs::Step2DArray>("/robotis_" + std::to_string(robot_id) + "/online_walking/footsteps_2d", 1);
    
    // Services
    get_param_client_ = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis_" + std::to_string(robot_id) + "/walking/get_params");
    footstep_planner_client_ = nh.serviceClient<localization_pkg::GetRelativeFootsteps>("/robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner");
}

WalkingController::~WalkingController() {}
  
void WalkingController::goWalk(std::string& command, bool default_walk)
{
    if (this->getModule("r_knee") != "walking_module")
    {
        this->setModule("walking_module");
    }

    if (command == "start" && default_walk) 
    {
        setWalkingParam(0.0125, 0, 0, true);  // 1.25 cm/step | y-place (lateral) | theta (rot) | balance
    }

    std_msgs::String command_msg;
    command_msg.data = command;
    walk_command_pub.publish(command_msg);
}

void WalkingController::goWalkSteps()
{
    if (this->getModule("r_knee") != "footstep_walking_module")
    {
        this->setModule("footstep_walking_module");
    }

    std_msgs::String command_msg;
    command_msg.data = start_walking_command_;
    footstep_walk_command_pub.publish(command_msg);
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
        ROS_INFO_COND(false, "Get walking parameters");  // true -> print
        return;
    } else {
        ROS_ERROR_LOG("Fail to get walking parameters", false);
        return;
    }
}

void WalkingController::startWalking(bool default_walk)
{
    goWalk(start_walking_command_, default_walk);
}

void WalkingController::stopWalking()
{
    goWalk(stop_walking_command_);
}

bool WalkingController::walkToPose(double x_goal, double y_goal, double theta_goal)
{
    std::vector<op3_online_walking_module_msgs::Step2D> steps;

    bool success = callFootstepPlanner(x_goal, y_goal, theta_goal, steps);
    if (success) 
    {
        ROS_COLORED_LOG("Footstep plan successful. Sending footsteps to walking module...", CYAN, false);
        std_msgs::String balance_enable_msg;
        balance_enable_msg.data = "balance_on";
        balance_enable_pub_.publish(balance_enable_msg);
        publishFootsteps(steps, 0.7); // 0.7s per step — tune as needed
        return true;
    } 
    else 
    {
        ROS_ERROR_LOG("Footstep planning failed", false);
        return false;
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
        ROS_ERROR_LOG("Failed to call /plan_footsteps service", false);
        return false;
    }

    if (!srv.response.result)
    {
        ROS_ERROR_LOG("Footstep planner returned 'false' for result", false);
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

    online_step_pub_.publish(msg);
}

bool WalkingController::walkFootstepPlan(const std::vector<humanoid_nav_msgs::StepTarget>& plan)
{
    for (size_t i = 1; i < plan.size(); ++i) {
        
        if (plan[i].leg != humanoid_nav_msgs::StepTarget::left)
            continue;  // skip if not left foot

        // Find the previous left foot step
        size_t prev = i - 1;
        while (prev > 0 && plan[prev].leg != humanoid_nav_msgs::StepTarget::left)
            --prev;

        if (plan[prev].leg != humanoid_nav_msgs::StepTarget::left)
            continue;  // can't find previous left step, skip

        double dx = plan[i].pose.x - plan[prev].pose.x;
        double dy = plan[i].pose.y - plan[prev].pose.y;
        double dtheta = plan[i].pose.theta - plan[prev].pose.theta;
        
        double forward = utils::clamp(dx, -0.1, 0.1);      // in-place forward motion
        double lateral = 0.0; // you can use dy if needed for sidesteps
        double angle = 0.0; //clamp(dtheta, -0.1, 0.1);        // turn slowly

        ROS_COLORED_LOG("forward: %f", CYAN, false, forward);
        ROS_COLORED_LOG("lateral: %f", CYAN, false, lateral);
        ROS_COLORED_LOG("angle: %f", CYAN, false, angle);
        setWalkingParam(forward, lateral, angle, true);
        startWalking(false);
        ros::Duration(1).sleep();   // 700 ms step duration
        stopWalking();
    }
    return true;
}

bool WalkingController::walkToGoalPose(double x_goal, double y_goal, double theta_goal)
{
    humanoid_nav_msgs::PlanFootsteps srv;

    // Set the request
    srv.request.start.x = 0.0;
    srv.request.start.y = 0.0;
    srv.request.start.theta = 0.0;

    srv.request.goal.x = x_goal;
    srv.request.goal.y = y_goal;
    srv.request.goal.theta = theta_goal;

    // Call the planner service
    if (!footstep_planner_client_.call(srv)) {
        ROS_ERROR_LOG("Failed to call /plan_footsteps service", false);
        return false;
    }

    // Check the result
    if (!srv.response.result) {
        ROS_ERROR_LOG("Planner failed to produce a plan", false);
        return false;
    }

    // output each footstep
    ROS_INFO("[WalkingController] Footstep Plan Output:");
    for (size_t i = 0; i < srv.response.footsteps.size(); ++i) {
        const auto& step = srv.response.footsteps[i];
        const char* foot = (step.leg == humanoid_nav_msgs::StepTarget::left) ? "LEFT" : "RIGHT";
        ROS_INFO("Step %2lu: foot=%s, x=%.3f, y=%.3f, theta=%.3f",
                 i, foot, step.pose.x, step.pose.y, step.pose.theta);
    }

    // Optionally print how many footsteps were generated
    ROS_COLORED_LOG("Planner succeeded. Steps: %lu", CYAN, false, srv.response.footsteps.size());

    // Walk the planned footsteps
    return walkFootstepPlan(srv.response.footsteps);
}


std::vector<humanoid_nav_msgs::StepTarget> WalkingController::callSoccerLocalizationService(
    double start_x, double start_y, double start_theta,
    double goal_x, double goal_y, double goal_theta)
{
    localization_pkg::GetRelativeFootsteps srv;

    // Fill request
    srv.request.start_x = start_x;
    srv.request.start_y = start_y;
    srv.request.start_theta = start_theta;
    srv.request.goal_x = goal_x;
    srv.request.goal_y = goal_y;
    srv.request.goal_theta = goal_theta;

    std::vector<humanoid_nav_msgs::StepTarget> relative_plan;

    if (footstep_planner_client_.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Service call succeeded");
            ROS_INFO("Relative plan has %zu steps", srv.response.relative_plan.size());

            relative_plan = srv.response.relative_plan;

            for (size_t i = 0; i < relative_plan.size(); ++i)
            {
            const auto& step = relative_plan[i];
            ROS_INFO("Relative Step %zu: x=%.3f y=%.3f theta=%.3f leg=%d",
                        i,
                        step.pose.x,
                        step.pose.y,
                        step.pose.theta);
            }
        }
        else
        {
        ROS_WARN("Service returned failure");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /call_footstep_planner");
    }
    return relative_plan;
}

