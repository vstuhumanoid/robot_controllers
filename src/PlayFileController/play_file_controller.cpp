//
// Created by humanoid on 27.04.18.
//

#include <ros/ros.h>
#include "JointNameConverter.h"
#include "PlayFileActionServer/PlayFileActionServer.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "play_file_controller");
    ROS_INFO("i am started...");
    ros::NodeHandle nh;

    // Get parameters
    int execute_rate;

    if(!nh.getParam("execute_rate", execute_rate))
    {
        ROS_ERROR("Required param \"execute_rate\" not set");
        return -1;
    }

    PlayFileActionServer action(nh, ros::this_node::getName(), execute_rate);

    ros::spin();
    return 0;
}