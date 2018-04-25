//
// Created by skorikov on 25.04.18.
//

#include "FrundModelActionServer.h"

using namespace robot_controllers;
using namespace std;
using namespace robot_msgs;

FrundModelActionServer::FrundModelActionServer(ros::NodeHandle& nh, string server_name) :
    as_(nh, server_name, boost::bind(&FrundModelActionServer::execute_cb, this, _1), false),
    nh_(nh),
    ac_("transition_controller", true)
{
    joints_mode_publisher_ = nh_.advertise<JointsMode>("/joints/set_mode", 100);
    joints_command_publisher_ = nh_.advertise<JointsCommand>("/joints/commands", 1000);
    state_subscriber_ = nh_.subscribe("/joints/state", 1000, &FrundModelActionServer::joints_state_cb, this);
    params_subscriber_ = nh_.subscribe("/joints/get_params", 100, &FrundModelActionServer::joints_params_cb, this);
    motion_params_subscriber_ = nh_.subscribe("/frund/motion_params", 1000, &FrundModelActionServer::motion_params_cb, this);

    as_.start();
}

bool FrundModelActionServer::initializeGoal(FrundModelGoalConstPtr goal)
{

}

void FrundModelActionServer::execute_transition()
{
    //TODO: вызов перехода (передавать цель или позиции?)
}

void FrundModelActionServer::execute_cb(const FrundModelGoalConstPtr &goal)
{

}

/*void FrundModelActionServer::preemptCB()
{
    // Либо отправлена новая задача, либо текущая задача отменена
    ROS_WARN_STREAM("preempt_cb");
}*/

void FrundModelActionServer::joints_state_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointState_ = *msg;
}

void FrundModelActionServer::joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointsParams_ = *msg;
}

void FrundModelActionServer::motion_params_cb(const robot_controllers::MotionParamsConstPtr &msg)
{
    MotionParams motionParams = *msg;

    //TODO: Передача ФРУНДУ
}