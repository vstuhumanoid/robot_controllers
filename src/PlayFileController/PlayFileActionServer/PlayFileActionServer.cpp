//
// Created by humanoid on 27.04.18.
//

#include "PlayFileActionServer.h"

using namespace robot_msgs;

PlayFileActionServer::PlayFileActionServer(ros::NodeHandle &nh, std::string server_name, int execute_rate) :
        as_(nh, server_name, boost::bind(&PlayFileActionServer::execute_cb, this, _1), false),
        execute_rate_(execute_rate),
        nh_(nh),
        ac_("transition_controller", true)
{
    joints_mode_publisher_ = nh_.advertise<JointsMode>("/joints/set_mode", 100);
    joints_command_publisher_ = nh_.advertise<JointsCommand>("/joints/commands", 1000);

    params_subscriber_ = nh_.subscribe("/joints/get_params", 100, &PlayFileActionServer::joints_params_cb, this);

    //ac_.waitForServer();
    as_.start();
}

void PlayFileActionServer::execute_cb(const robot_controllers::PlayFileGoalConstPtr &goal)
{

}

bool PlayFileActionServer::initializeGoal(robot_controllers::PlayFileGoalConstPtr goal)
{
    return false;
}

void PlayFileActionServer::execute_step()
{

}

void PlayFileActionServer::joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg)
{

}