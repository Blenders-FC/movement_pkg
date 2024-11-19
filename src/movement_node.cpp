/* Author: Pedro Deniz */

#include <std_srvs/Empty.h>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Eigen>

// #include "robotis_controller/robotis_controller.h"
#include "robotis_controller_msgs/SetModule.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "op3_action_module_msgs/IsRunning.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingParam.h"

void readyToDemo();
void setModule(const std::string& module_name);
bool checkManagerRunning(std::string& manager_name);
void torqueOnAll();

void goInitPose();
void goAction(int page);
void goWalk(std::string& command);

bool isActionRunning();
bool getWalkingParam();
void publishHeadJoint(double pan, double tilt);
void walkTowardsBall(double pan, double tilt);

void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle);
void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance = true);

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg);
void callbackJointStates(const sensor_msgs::JointState& msg);

double rpy_orientation;
const double FALL_FORWARD_LIMIT = 55;
const double FALL_BACK_LIMIT = -55;
double present_pitch_;
double head_pan;
double head_tilt;
double distance_to_ball = 0.0;
int state;

const double SPOT_FB_OFFSET = 0.0;
const double SPOT_RL_OFFSET = 0.0;
const double SPOT_ANGLE_OFFSET = 0.0;
double current_x_move_ = 0.005;
double current_r_angle_ = 0.0;
const double IN_PLACE_FB_STEP = -0.003;
const double CAMERA_HEIGHT = 0.46;
const double FOV_WIDTH = 35.2 * M_PI / 180;
const double FOV_HEIGHT = 21.6 * M_PI / 180;
const double hip_pitch_offset_ = 0.12217305;  //7°
const double UNIT_FB_STEP = 0.002;
const double UNIT_RL_TURN = 0.00872665;  //0.5°
const double MAX_FB_STEP = 0.007;
const double MAX_RL_TURN =  0.26179939;  //15°
const double MIN_FB_STEP = 0.003;
const double MIN_RL_TURN = 0.08726646;  //5°
double accum_period_time = 0.0;
double current_period_time = 0.6;

const int SPIN_RATE = 30;
const bool DEBUG_PRINT = false;

std::string command;
sensor_msgs::JointState head_angle_msg;
ros::Time prev_time_walk;

enum ControlModule
{
  None = 0,
  DirectControlModule = 1,
  Framework = 2,
};

ros::Publisher init_pose_pub;
ros::Publisher dxl_torque_pub;
ros::Publisher action_pose_pub;
ros::Publisher walk_command_pub;
ros::Publisher set_walking_param_pub;
ros::Publisher write_head_joint_offset_pub;

ros::Subscriber read_joint_sub;
ros::Subscriber imu_sub;

ros::ServiceClient set_joint_module_client;
ros::ServiceClient is_running_client;
ros::ServiceClient get_param_client;

op3_walking_module_msgs::WalkingParam current_walking_param;

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "read_write");
    ros::NodeHandle nh(ros::this_node::getName());

    int robot_id;
    nh.param<int>("robot_id", robot_id, 0);

    //subscribers
    read_joint_sub = nh.subscribe("/robotis/present_joint_states",1, callbackJointStates);
    imu_sub = nh.subscribe("/robotis/open_cr/imu", 1, callbackImu);
    
    
    //publishers
    init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
    dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
    action_pose_pub = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
    walk_command_pub = nh.advertise<std_msgs::String>("/robotis/walking/command", 0);
    write_head_joint_offset_pub = nh.advertise<sensor_msgs::JointState>("/robotis/head_control/set_joint_states", 0);
    set_walking_param_pub = nh.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);

    //services
    set_joint_module_client = nh.serviceClient<robotis_controller_msgs::SetModule>("/robotis/set_present_ctrl_modules");
    is_running_client = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
    get_param_client = nh.serviceClient<op3_walking_module_msgs::GetWalkingParam>("/robotis/walking/get_params");

    ros::start();

    //set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    //wait for starting of op3_manager
    std::string manager_name = "/op3_manager";
    while (ros::ok())
    {
        ros::Duration(1.0).sleep();

        if (checkManagerRunning(manager_name) == true)
        {
        break;
        ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
        }
        ROS_WARN("Waiting for op3 manager");
    }

    readyToDemo();
    
    //pararse en posición para caminar
    ros::Duration(1).sleep();
    ros::Rate loop_rate_pararse(60);
    
    goAction(9);
    
    setModule("head_control_module");
    head_angle_msg.name.push_back("head_pan");
    head_angle_msg.position.push_back(0.5236);  // 30°
    head_angle_msg.name.push_back("head_tilt");
    head_angle_msg.position.push_back(-0.3491);       // 20°
    write_head_joint_offset_pub.publish(head_angle_msg);

    ros::Duration(3.0).sleep();
    ros::Time prev_time_walk = ros::Time::now();
    setModule("walking_module");
    walkTowardsBall(head_pan, head_tilt);
    
    // while (ros::ok()){
    //     ros::Rate loop_rate(SPIN_RATE);
    //     ros::spinOnce();
            
    //     setModule("walking_module");
        
    // }
    return 0;
}

void readyToDemo()
{
  ROS_INFO("Start read-write demo");
  torqueOnAll();
  ROS_INFO("Torque on all joints");

  //send message for going init posture
  goInitPose();
  ROS_INFO("Go init pose");

  //wait while ROBOTIS-OP3 goes to the init posture.
  ros::Duration(4.0).sleep();

  setModule("none");
}

void goInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";
  init_pose_pub.publish(init_msg);
}

void goAction(int page) 
{
  setModule("action_module");
  ROS_INFO("Action pose");

  std_msgs::Int32 action_msg;
  action_msg.data = page;
  action_pose_pub.publish(action_msg);
}

void goWalk(std::string& command) 
{
  setModule("walking_module");
  if (command == "start") {
    getWalkingParam();
    setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
  }

  std_msgs::String command_msg;
  command_msg.data = command;
  walk_command_pub.publish(command_msg);
}

void publishHeadJoint(double pan, double tilt) {
  if (pan >= 1.2217) pan = 1.2217;            //70 deg
  else if (pan <= -1.2217) pan = -1.2217;     //-70 deg
  
  if (tilt >= 0.34906) tilt = 0.34906;        //20 deg
  else if (tilt <= -1.2217) tilt = -1.2217;   //-70 deg
  
  head_angle_msg.name.push_back("head_pan");
  head_angle_msg.position.push_back(pan);
  head_angle_msg.name.push_back("head_tilt");
  head_angle_msg.position.push_back(tilt);

  write_head_joint_offset_pub.publish(head_angle_msg);
}

void walkTowardsBall(double pan, double tilt){
  ros::Time curr_time_walk = ros::Time::now();
  ros::Duration dur_walk = curr_time_walk - prev_time_walk;
  double delta_time_walk = dur_walk.nsec * 0.000000001 + dur_walk.sec;
  prev_time_walk = curr_time_walk;

  distance_to_ball = CAMERA_HEIGHT * tan(M_PI * 0.5 + tilt - hip_pitch_offset_);

  if (distance_to_ball < 0) {
    distance_to_ball *= (-1);
  }

  double distance_to_kick = 0.22;  //0.22

  // std::cout << distance_to_ball << std::endl;

  double fb_move = 0.0, rl_angle = 0.0;
  double distance_to_walk = distance_to_ball - distance_to_kick;
  
  // if (pan > 0.262){
  //   turn2search(2);
  //   std::string command = "stop";
  //   goWalk(command);
  //   ros::Duration(1).sleep();
  // }else if (pan < -0.262){
  //   turn2search_left(2); 
  //   std::string command = "stop";
  //   goWalk(command);
  //   ros::Duration(1).sleep();
  // }else{
    calcFootstep(distance_to_walk, pan, delta_time_walk, fb_move, rl_angle);

    getWalkingParam();
    setWalkingParam(fb_move, 0, rl_angle, true);
    
    std_msgs::String command_msg;
    command_msg.data = "start";
    walk_command_pub.publish(command_msg);
  //}
}

void calcFootstep(double target_distance, double target_angle, double delta_time, double& fb_move, double& rl_angle) 
{
  double next_movement = current_x_move_;
  if (target_distance < 0)
    target_distance = 0.0;

  double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
  accum_period_time += delta_time;
  if (accum_period_time > (current_period_time  / 4))
  {
    accum_period_time = 0.0;
    if ((target_distance * 0.1 / 2) < current_x_move_)
      next_movement -= UNIT_FB_STEP;
    else
      next_movement += UNIT_FB_STEP;
  }
  fb_goal = fmin(next_movement, fb_goal);
  fb_move = fmax(fb_goal, MIN_FB_STEP);

  double rl_goal = 0.0;
  if (fabs(target_angle) * 180 / M_PI > 5.0)
  {
    double rl_offset = fabs(target_angle) * 0.2;
    rl_goal = fmin(rl_offset, MAX_RL_TURN);
    rl_goal = fmax(rl_goal, MIN_RL_TURN);
    rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

    if (target_angle < 0)
      rl_angle *= (-1);
  }
}

bool checkManagerRunning(std::string& manager_name) 
{
  std::vector<std::string> node_list;
  ros::master::getNodes(node_list);

  for (unsigned int node_list_idx = 0; node_list_idx < node_list.size(); node_list_idx++)
  {
    if (node_list[node_list_idx] == manager_name)
      return true;
  }
  ROS_ERROR("Can't find op3_manager");
  return false;
}

void setModule(const std::string& module_name) 
{
  // RobotisController *controller = RobotisController::getInstance();
  // controller->setCtrlModule(module_name);
  // usleep(200 * 1000);
  // return;
  robotis_controller_msgs::SetModule set_module_srv;
  set_module_srv.request.module_name = module_name;

  if (set_joint_module_client.call(set_module_srv) == false)
  {
    ROS_ERROR("Failed to set module");
    return;
  }
  return ;
}

void torqueOnAll() 
{
  std_msgs::String check_msg;
  check_msg.data = "check";
  dxl_torque_pub.publish(check_msg);
}

bool isActionRunning() 
{
  op3_action_module_msgs::IsRunning is_running_srv;

  if (is_running_client.call(is_running_srv) == false) {
    ROS_ERROR("Failed to start action module");
    return true;
  } else {
    if (is_running_srv.response.is_running == true) {
      return true;
    }
  }
  return false;
}

void setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
{
  current_walking_param.balance_enable = balance;
  current_walking_param.x_move_amplitude = x_move + SPOT_FB_OFFSET;
  current_walking_param.y_move_amplitude = y_move + SPOT_RL_OFFSET;
  current_walking_param.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

  set_walking_param_pub.publish(current_walking_param);

  current_x_move_ = x_move;
  current_r_angle_ = rotation_angle;
}

bool getWalkingParam() 
{
  
  op3_walking_module_msgs::GetWalkingParam walking_param_msg;

  if (get_param_client.call(walking_param_msg))
  {
    current_walking_param = walking_param_msg.response.parameters;

    // update ui
    ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");

    return true;
  }
  else
  {
    ROS_ERROR("Fail to get walking parameters.");

    return false;
  }
}

void callbackJointStates(const sensor_msgs::JointState& msg)
{ 
  head_pan = msg.position[0];
  head_tilt = msg.position[1];
  return;
}

void callbackImu(const sensor_msgs::Imu::ConstPtr& msg) 
{
  Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  rpy_orientation *= (180 / 3.141516);

  double pitch = rpy_orientation.coeff(1, 0);

  double alpha = 0.4;
  if (present_pitch_ == 0) 
    present_pitch_ = pitch;
  else
    present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  if (present_pitch_ > FALL_FORWARD_LIMIT) {
    goAction(122);
    setModule("none");
  } else if (present_pitch_ < FALL_BACK_LIMIT) {
    goAction(1);
    setModule("none");
    ros::Duration(1).sleep();
    goAction(82);
    setModule("none");
  } else {
    state = 0;
  }
}
