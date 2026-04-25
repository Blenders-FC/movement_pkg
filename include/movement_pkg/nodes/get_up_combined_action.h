/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#ifndef GET_UP_COMBINED_ACTION_H
#define GET_UP_COMBINED_ACTION_H

#include "movement_pkg/utils.h"
#include "behaviortree_cpp/action_node.h"
#include "movement_pkg/cb_data_manager.h"

namespace BT
{
class GetUpCombined : public BT::StatefulActionNode, public CBDataManager
{
public:
    // Constructor
    explicit GetUpCombined(const std::string& name, const NodeConfig& config);
    ~GetUpCombined();

    // The method that is going to be executed by the thread
    NodeStatus onStart() override;
    NodeStatus onRunning() override;

    // The method used to interrupt the execution of the node
    void onHalted();

    static BT::PortsList providedPorts();
private:
    std::shared_ptr<utils> utils_;
    rclcpp::Node::SharedPtr node_;
    double pitch;
    double alpha = 0.4;
    double present_pitch_ = 0;
    const double FALL_FORWARD_LIMIT = 55;
    const double FALL_BACKWARDS_LIMIT = -55;
};
}  // namespace BT

#endif  // GET_UP_COMBINED_ACTION_H
