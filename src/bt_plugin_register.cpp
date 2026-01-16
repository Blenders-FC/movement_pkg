/*
    Authors:
        Ricardo Berumen

*/

#include "movement_pkg/bt_plugin_register.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<BT::ManagerRunningCondition>("ManagerRunning");
    factory.registerNodeType<BT::LeftKick>("LeftKick");
    factory.registerNodeType<BT::SimpleWalk>("SimpleWalk");
}