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

    FrundModelActionServer action_server(nh, ros::this_node::getName());

    ros::spin();
    return 0;
}