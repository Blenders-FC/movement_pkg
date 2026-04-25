/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include "movement_pkg/nodes/get_up_combined_action.h"


namespace BT
{
GetUpCombined::GetUpCombined(
    const std::string &name, 
    const BT::NodeConfig &config)
:StatefulActionNode(name, config)
{
    //type_ = BT::ACTION_NODE;
    //thread_ = std::thread(&GetUpCombined::WaitForTick, this);
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("GetUpCombined: missing [node] in blackboard");
}
    utils_ = std::make_shared<utils>(node_);
    RCLCPP_INFO(node_->get_logger(), "GetUpCombined constructed");
}

BT::GetUpCombined::~GetUpCombined() {}

NodeStatus BT::GetUpCombined::onStart()
{
    RCLCPP_INFO(node_->get_logger(), "[GetUpCombined] START");
    return NodeStatus::RUNNING;
}

NodeStatus BT::GetUpCombined::onRunning()
{
        // Waiting for the first tick to come
        //ROS_TAGGED_ONCE_LOG("WAIT FOR TICK", "DEFAULT", false, "Wait_getup_comb");
        //tick_engine.Wait();
        //ROS_TAGGED_ONCE_LOG("TICK RECEIVED", "DEFAULT", false, "Received_getup_comb");
        RCLCPP_INFO(node_->get_logger(), "Waiting for tick...");
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // Sleep to
        // Perform action...



            utils_->goAction(1);  // straighten legs
            rclcpp::sleep_for(std::chrono::milliseconds(500));

            pitch = getRobotPitch();
        
            if (present_pitch_ == 0) 
                present_pitch_ = pitch;
            else
                present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;


            if (present_pitch_ > FALL_FORWARD_LIMIT)
            {
                RCLCPP_INFO(node_->get_logger(), "Forward fall detected with pitch: %f", present_pitch_);
                utils_->goAction(122);  // get up forward
                rclcpp::sleep_for(std::chrono::seconds(1));

                RCLCPP_INFO(node_->get_logger(), "Get up forwards action");
                //set_status(BT::SUCCESS);
                return BT::NodeStatus::SUCCESS;
            }
            else if (present_pitch_ < FALL_BACKWARDS_LIMIT) 
            {
                RCLCPP_INFO(node_->get_logger(), "Backwards fall detected with pitch: %f", present_pitch_);
                utils_->goAction(82);  // get up forward
                rclcpp::sleep_for(std::chrono::seconds(1));

                RCLCPP_INFO(node_->get_logger(), "Get up backwards action");
                //set_status(BT::SUCCESS);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Fall not detected");
                //set_status(BT::FAILURE);
                return BT::NodeStatus::FAILURE;
            }

    RCLCPP_ERROR(node_->get_logger(), "ROS stopped unexpectedly");
    return BT::NodeStatus::FAILURE;
}

BT::PortsList BT::GetUpCombined::providedPorts()
{
    return {};
}

void BT::GetUpCombined::onHalted()
{
    //set_status(BT::HALTED);
    RCLCPP_WARN(node_->get_logger(), "GetUpCombined HALTED: Stopped get up combined");
}
}  // namespace BT