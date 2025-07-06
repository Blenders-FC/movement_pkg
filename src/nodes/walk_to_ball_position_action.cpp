/*
    Authors:
        Pedro Deniz
*/

#include "movement_pkg/nodes/walk_to_ball_position_action.h"


BT::WalkToBallPosition::WalkToBallPosition(std::string name) 
: ActionNode::ActionNode(name), WalkingController()
{
    type_ = BT::ACTION_NODE;
    thread_ = std::thread(&WalkToBallPosition::WaitForTick, this);

    ros::ServiceClient client = nh.serviceClient<localization_pkg::GetRelativeFootsteps>(
        "/robotis_" + std::to_string(robot_id) + "/soccer_localization_node/call_footstep_planner");
    
    leg_plan_pub_ = nh.advertise<footstep_walking_module::FootstepLegPlan>("/robotis_" + std::to_string(robot_id) + "/leg_step_plan", 1);
}

BT::WalkToBallPosition::~WalkToBallPosition() {}

void BT::WalkToBallPosition::WaitForTick()
{
    while(ros::ok())
    {
        ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_walk_point");
        tick_engine.Wait();
        ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_walk_point");

        // Perform action...
        while (get_status() == BT::IDLE)
        {
            // set_status(BT::RUNNING);

            // if (ball && std::isfinite(ball->distance) && std::isfinite(ball->pan_angle)
            //     && ball->distance > 0 && std::abs(ball->pan_angle) <= M_PI)
            // {
            //     distance_to_ball = ball->distance;
            //     pan_angle_to_ball = ball->pan_angle;
            //     ROS_COLORED_LOG("Ball data loaded from blackboard!", GREEN, false);
            // }
            // else
            // {
            //     ROS_COLORED_LOG("Invalid or missing ball data. Using default values", ORANGE, false);
            // }

            this->setModule("footstep_walking_module");
            ROS_COLORED_LOG("Walking towards ball in coordinates: X=%.4f Y=%.4f theta=%.4f", CYAN, true, distance_to_ball, pan_angle_to_ball, pan_angle_to_ball);
            
            
            
            auto walking_plan = callSoccerLocalizationService(start_x, start_y, start_theta, goal_x, goal_y, goal_theta);
            walkingSucced = publishLegPlan(walking_plan);

            if (walkingSucced)
            {
                ROS_SUCCESS_LOG("Walking to ball process has finished successfully!");
                set_status(BT::SUCCESS);
            }
            else{
                ROS_ERROR_LOG("Walk to ball FAILED", false);
                set_status(BT::FAILURE);
            }
        }
    }
    ROS_ERROR_LOG("ROS stopped unexpectedly", false);
    set_status(BT::FAILURE);
}

bool BT::WalkToBallPosition::publishLegPlan(const std::vector<humanoid_nav_msgs::StepTarget>& walking_plan, ros::Publisher& pub)
{
    footstep_walking_module::FootstepLegPlan plan_msg;

    if (walking_plan.empty())
    {
        ROS_WARN("Walking plan is empty, nothing to publish");
        return false;
    }

    for (const auto& step : walking_plan)
    {
        footstep_walking_module::FootstepLegStep step_msg;
        step_msg.x_move_amplitude = step.pose.x;
        step_msg.y_move_amplitude = step.pose.y;
        step_msg.angle_move_amplitude = step.pose.theta;
        plan_msg.steps.push_back(step_msg);
    }

    pub.publish(plan_msg);
    ROS_INFO("Published leg plan with %zu steps for leg: %s",
            plan_msg.steps.size(), plan_msg.leg.c_str());
    
    return true;
}

void BT::WalkToBallPosition::Halt()
{
    stopWalking();

    set_status(BT::HALTED);
    ROS_TAGGED_ONCE_LOG("WalkToBallPosition HALTED: Stopped walking towards ball", "ORANGE", false, "Halted_walk_ball");
}
