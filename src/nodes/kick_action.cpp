/*
    Authors:
        Pedro Deniz
        Marlene Cobian
*/

#include <movemente_pkg/nodes/kick_action.h>

BT::KickAction::KickAction(std::string name) : ActionNode::ActionNode(name)
{
    type_ = BT::ACTION_NODE;

    // Publishers
    action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis_" + std::to_string(robot_id) + "/action/page_num", 0);


    thread_ = std::thread(&KickAction::WaitForTick, this);
}

BT::KickAction::~KickAction() {}

void BT::KickAction::WaitForTick()
{
    while (true)
    {
        // Waiting for the first tick to come
        DEBUG_STDOUT(get_name() << " WAIT FOR TICK");
        tick_engine.Wait();
        DEBUG_STDOUT(get_name() << " TICK RECEIVED");

        // Running state
        set_status(BT::RUNNING);

        // Perform action...
        if (pan > 0) { //left
            std::cout << "PATEA DERECHA" << std::endl;
            goAction(84); //left kick
        }
        else { //right
            std::cout << "PATEA IZQUIERDA" << std::endl;
            goAction(83); //right kick
        }

        DEBUG_STDOUT(get_name() << "Kick action SUCCESS");
        set_status(BT::SUCCESS);
    }
}

void goAction(int page) {
    setModule("action_module");
    ROS_INFO("Action pose");
  
    std_msgs::Int32 action_msg;
    action_msg.data = page;
    action_pose_pub.publish(action_msg);
  }