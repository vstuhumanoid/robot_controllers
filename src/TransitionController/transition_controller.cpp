//
// Created by humanoid on 09.04.18.
//

#include <ros/ros.h>
#include "TransitionActionServer/TransitionActionServer.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transition_controller");
    ROS_INFO("i am started...");
    ros::NodeHandle nh;

    // Get parameters
    double transition_speed;
    int execute_rate;
    if(!nh.getParam("transition_speed", transition_speed))
    {
        ROS_ERROR("Required param \"transition_speed\" not set");
        return -1;
    }

    if(!nh.getParam("execute_rate", execute_rate))
    {
        ROS_ERROR("Required param \"execute_rate\" not set");
        return -1;
    }

    TransitionActionServer action(nh, ros::this_node::getName(), execute_rate, transition_speed);

    ros::spin();
    return 0;
}

