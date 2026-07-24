/*
    Authors:
        Ricardo Berumen

*/

#include "movement_pkg/bt_plugin_register.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<BT::BallDetectedCondition>("BallDetectedCondition");
    factory.registerNodeType<BT::CenterBallYOLOPID>("CenterBallYOLOPID");
    factory.registerNodeType<BT::ChooseKickFootCondition>("ChooseKickFootCondition");
    factory.registerNodeType<BT::GetUpCombined>("GetUpCombined");
    factory.registerNodeType<BT::LeftKick>("LeftKick");
    factory.registerNodeType<BT::ManagerDoneCondition>("ManagerDoneCondition");
    factory.registerNodeType<BT::ManagerRunningCondition>("ManagerRunning");
    factory.registerNodeType<BT::RightKick>("RightKick");
    factory.registerNodeType<BT::RobotFallenCondition>("RobotFallenCondition");
    factory.registerNodeType<BT::SearchBallSinusoidal>("SearchBallSinusoidal");
    factory.registerNodeType<BT::HeadToHome>("HeadToHome");    
    factory.registerNodeType<BT::HeadToHomeReset>("HeadToHomeReset");
    factory.registerNodeType<BT::SimpleWalk>("SimpleWalk");
    factory.registerNodeType<BT::StandUp>("StandUp");
    factory.registerNodeType<BT::StartButtonCondition>("StartButtonCondition");
    factory.registerNodeType<BT::WalkToTarget>("WalkToTarget");
    factory.registerNodeType<BT::RefereeStateCondition>("RefereeStateCondition");
}