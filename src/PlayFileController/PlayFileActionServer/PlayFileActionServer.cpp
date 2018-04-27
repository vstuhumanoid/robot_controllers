//
// Created by humanoid on 27.04.18.
//

#include "PlayFileActionServer.h"

PlayFileActionServer::PlayFileActionServer(ros::NodeHandle &nh, std::string server_name, int execute_rate) :
        as_(nh, server_name, boost::bind(&PlayFileActionServer::execute_cb, this, _1), false),
        execute_rate_(execute_rate),
        nh_(nh),
        ac_("transition_controller", true)
{
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
