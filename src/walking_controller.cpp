/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movement_pkg/walking_controller.h>


WalkingController::WalkingController(rclcpp::Node::SharedPtr node) : utils(node)
{
    // Publishers
    walk_command_pub =
        node_->create_publisher<std_msgs::msg::String>(
            "/robotis_" + std::to_string(robot_id) + "/walking/command",
            10);

    set_walking_param_pub_ =
        node_->create_publisher<op3_walking_module_msgs::msg::WalkingParam>(
            "/robotis_" + std::to_string(robot_id) + "/walking/set_params",
            10);

    balance_enable_pub_ =
        node_->create_publisher<std_msgs::msg::String>(
            "/robotis_" + std::to_string(robot_id) + "/online_walking/wholebody_balance_msg",
            1);

    online_step_pub_ =
        node_->create_publisher<op3_online_walking_module_msgs::msg::Step2DArray>(
            "/robotis_" + std::to_string(robot_id) + "/online_walking/footsteps_2d",
            1);

    // Services
    get_param_client_ =
        node_->create_client<op3_walking_module_msgs::srv::GetWalkingParam>(
            "/robotis_" + std::to_string(robot_id) + "/walking/get_params");

    footstep_planner_client_ = 
        node_->create_client<humanoid_nav_msgs::srv::PlanFootsteps>(
            "/robotis_" + std::to_string(robot_id) + "/plan_footsteps");
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

    std_msgs::msg::String command_msg;
    command_msg.data = command;
    walk_command_pub->publish(command_msg);
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

    set_walking_param_pub_->publish(current_walking_param_);

    current_x_move_ = x_move;
    current_r_angle_ = rotation_angle;
}

void WalkingController::getWalkingParam()
{
    if (!get_param_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "WalkingParam service not available");
        return;
    }

    auto request =
        std::make_shared<op3_walking_module_msgs::srv::GetWalkingParam::Request>();

    auto future = get_param_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            node_, future, std::chrono::seconds(1))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to call GetWalkingParam service");
        return;
    }

    current_walking_param_ = future.get()->parameters;

    RCLCPP_INFO(node_->get_logger(), "Get walking parameters");
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
    std::vector<op3_online_walking_module_msgs::msg::Step2D> steps;

    bool success = callFootstepPlanner(x_goal, y_goal, theta_goal, steps);
    if (success) 
    {
        RCLCPP_INFO(node_->get_logger(), "Footstep plan successful. Sending footsteps to walking module...");
        std_msgs::msg::String balance_enable_msg;
        balance_enable_msg.data = "balance_on";
        balance_enable_pub_->publish(balance_enable_msg);
        publishFootsteps(steps, 0.7); // 0.7s per step — tune as needed
        return true;
    } 
    else 
    {
        RCLCPP_ERROR(node_->get_logger(), "Footstep planning failed");
        return false;
    }
}

bool WalkingController::callFootstepPlanner(double x_goal, double y_goal, double theta_goal, std::vector<op3_online_walking_module_msgs::msg::Step2D>& step_list)
{
    humanoid_nav_msgs::srv::PlanFootsteps srv;

    if (!footstep_planner_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Service /plan_footsteps not available");
        return false;
    }

    auto request = std::make_shared<humanoid_nav_msgs::srv::PlanFootsteps::Request>();
    
    // Set start and goal in robot-local frame
    request->start.x = 0.0;
    request->start.y = 0.0;
    request->start.theta = 0.0;

    request->goal.x = x_goal;
    request->goal.y = y_goal;
    request->goal.theta = theta_goal;

    auto future = footstep_planner_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            node_, future, std::chrono::seconds(1))
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call /plan_footsteps service");
        return false;
    }

    auto response = future.get();

    if (!response->result)
    {
        RCLCPP_ERROR(node_->get_logger(), "Footstep planner failed to produce a plan");
        RCLCPP_INFO(node_->get_logger(), "Footstep planner returned 'false' for result");
        return false;
    }

    for (const auto& step : response->footsteps)
    {
        op3_online_walking_module_msgs::msg::Step2D step_msg;
        step_msg.step2d.x = step.pose.x;
        step_msg.step2d.y = step.pose.y;
        step_msg.step2d.theta = step.pose.theta;

        // Convert leg type from planner to OP3 walking module
        if (step.leg == humanoid_nav_msgs::msg::StepTarget::LEFT)
            step_msg.moving_foot = 0;  // LEFT_FOOT_SWING

        else if (step.leg == humanoid_nav_msgs::msg::StepTarget::RIGHT)
            step_msg.moving_foot = 1;  // RIGHT_FOOT_SWING

        else
            continue;  // skip unknown leg

        step_list.push_back(step_msg);
    }

    return true;
}

void WalkingController::publishFootsteps(const std::vector<op3_online_walking_module_msgs::msg::Step2D>& steps, double step_time)
{
    op3_online_walking_module_msgs::msg::Step2DArray msg;
    msg.step_time = step_time;

    for (const auto& s : steps)
        msg.footsteps_2d.push_back(s);

    online_step_pub_->publish(msg);
}

bool WalkingController::walkFootstepPlan(const std::vector<humanoid_nav_msgs::msg::StepTarget>& plan)
{
    for (size_t i = 1; i < plan.size(); ++i) {
        
        if (plan[i].leg != humanoid_nav_msgs::msg::StepTarget::LEFT)
            continue;  // skip if not left foot

        // Find the previous left foot step
        size_t prev = i - 1;
        while (prev > 0 && plan[prev].leg != humanoid_nav_msgs::msg::StepTarget::LEFT)
            --prev;

        if (plan[prev].leg != humanoid_nav_msgs::msg::StepTarget::LEFT)
            continue;  // can't find previous left step, skip

        double dx = plan[i].pose.x - plan[prev].pose.x;
        double dy = plan[i].pose.y - plan[prev].pose.y;
        double dtheta = plan[i].pose.theta - plan[prev].pose.theta;
        
        double forward = utils::clamp(dx, -0.1, 0.1);      // in-place forward motion
        double lateral = 0.0; // you can use dy if needed for sidesteps
        double angle = 0.0; //clamp(dtheta, -0.1, 0.1);        // turn slowly

        RCLCPP_INFO(node_->get_logger(), "forward: %f", forward);
        RCLCPP_INFO(node_->get_logger(), "lateral: %f", lateral);
        RCLCPP_INFO(node_->get_logger(), "angle: %f", angle);
        setWalkingParam(forward, lateral, angle, true);
        startWalking(false);
        rclcpp::sleep_for(std::chrono::milliseconds(700));   // 700 ms step duration
        stopWalking();
    }
    return true;
}



bool WalkingController::walkToGoalPose(double x_goal, double y_goal, double theta_goal)
{
    humanoid_nav_msgs::srv::PlanFootsteps srv;
    if (!footstep_planner_client_->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Service /plan_footsteps not available");
        return false;
    }
    auto request = std::make_shared<humanoid_nav_msgs::srv::PlanFootsteps::Request>();

    // Set the request
    request->start.x = 0.0;
    request->start.y = 0.0;
    request->start.theta = 0.0;

    request->goal.x = x_goal;
    request->goal.y = y_goal;
    request->goal.theta = theta_goal;
    // Call the planner service
    auto future = footstep_planner_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call /plan_footsteps service");
        return false;
    }
    auto response = future.get();

    // Check the result
    if (!response->result) {
        RCLCPP_ERROR(node_->get_logger(), "Planner failed to produce a plan");
        return false;
    }

    // output each footstep
    RCLCPP_INFO(node_->get_logger(), "[WalkingController] Footstep Plan Output:");
    for (size_t i = 0; i < response->footsteps.size(); ++i) {
        const auto& step = response->footsteps[i];
        const char* foot = (step.leg == humanoid_nav_msgs::msg::StepTarget::LEFT) ? "LEFT" : "RIGHT";
        RCLCPP_INFO(node_->get_logger(), "Step %2lu: foot=%s, x=%.3f, y=%.3f, theta=%.3f",
                 i, foot, step.pose.x, step.pose.y, step.pose.theta);
    }

    // Optionally print how many footsteps were generated
    RCLCPP_INFO(node_->get_logger(), "Planner succeeded. Steps: %lu", response->footsteps.size());

    // Walk the planned footsteps
    return walkFootstepPlan(response->footsteps);
}



