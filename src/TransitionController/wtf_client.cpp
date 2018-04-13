#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bomj");
    ros::NodeHandle handle;

    actionlib::SimpleActionClient<robot_controllers::TransitionAction> ac("transition_controller", true);

    ROS_INFO_STREAM("Waiting for start");

    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    robot_controllers::TransitionGoal goal;
    ac.sendGoal(goal);


    /*bool is_zaebis = ac.waitForResult(ros::Duration(10));
    if(is_zaebis)
    {
        auto state = ac.getState();
        ROS_INFO_STREAM("Action finished: " << state.toString());
    }
    else
    {
        ROS_INFO_STREAM("Fuck off. Cancelling...");
        ac.sendGoal(goal);
        ac.waitForResult();
        auto state = ac.getState();
        ROS_INFO_STREAM("Action finished: " << state.toString());
    }*/

    return 0;
}