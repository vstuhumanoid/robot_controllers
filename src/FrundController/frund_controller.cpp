//
// Created by skorikov on 25.04.18.
//

#include <ros/ros.h>
#include "FrundModelActionServer/FrundModelActionServer.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "frund_controller");
    ROS_INFO("i am started...");
    ros::NodeHandle nh;

    int execute_rate;
    if(!nh.getParam("execute_rate", execute_rate))
    {
        ROS_ERROR("Required param \"execute_rate\" not set");
        return -1;
    }

    FrundModelActionServer action_server(nh, ros::this_node::getName(), execute_rate);

    ros::spin();
    return 0;
}