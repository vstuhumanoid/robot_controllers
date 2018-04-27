//
// Created by humanoid on 27.04.18.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/PlayFileAction.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <string>
#include <ros/topic.h>
#include <iostream>

using namespace std;
using namespace robot_controllers;

sensor_msgs::JointState jointState;

void send(actionlib::SimpleActionClient<PlayFileAction>& ac, string goal_filename)
{
    PlayFileGoal goal;

    goal.filename = goal_filename;

    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);

    bool is_ok = ac.waitForResult();    // ros::Duration(waiting_time));
    if(is_ok)
    {
        auto state = ac.getState();
        if(state.state_ == state.SUCCEEDED)
        {
            auto result = ac.getResult();
            ROS_INFO_STREAM("State: " << state.toString() << ", " << state.getText());
            ROS_INFO_STREAM("Result received - Succeeded");
        }
        else
        {
            ROS_WARN_STREAM("State: " << state.toString() << ", " << state.getText());
        }

    }
    else
    {
        ROS_WARN("Timeout");
        ac.cancelGoal();
        ac.waitForResult();
        auto state = ac.getState();
        auto result = ac.getResult();
        ROS_INFO_STREAM("State: " << state.toString() << ", " << state.getText());
        ROS_INFO_STREAM("Result received - Error");
    }
}

int main(int argc, char** argv)
{
    for(int i = 0; i < argc; i++)
        cout << argv[i] << "  ";

    if(argc != 2)
    {
        ROS_ERROR("Not enough arguments!");
        return -1;
    }

    ros::init(argc, argv, argv[1]);
    actionlib::SimpleActionClient<PlayFileAction> ac("play_file_controller", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Server started");

    string goal_filename = 0;
    while(ros::ok())
    {
        //send(ac, task_count, wait);
        cout << "Enter goal: ";
        cin >> goal_filename;
        send(ac, goal_filename);
    }

    return 0;
}
