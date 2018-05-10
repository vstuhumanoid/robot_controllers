#include <ros/ros.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/FrundModelAction.h>

using namespace std;
using namespace robot_controllers;

void send(actionlib::SimpleActionClient<FrundModelAction>& ac, string goal_model)
{
    FrundModelGoal goal;
    goal.model = goal_model;
    
    ROS_INFO("Sending goal...");
    ac.sendGoal(goal);

    bool is_ok = ac.waitForResult(); //ros::Duration(20));
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

int main(int argc, char **argv)
{
    for(int i = 0; i < argc; i++)
        cout << argv[i] << "  ";

    if(argc != 2)
    {
        ROS_ERROR("Not enough arguments!");
        return -1;
    }

    ros::init(argc, argv, argv[1]);
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<FrundModelAction> ac("frund_controller", true);

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Server started");

    string model_name;
    while(ros::ok())
    {
        //send(ac, task_count, wait);
        cout << "Enter goal: ";
        cin >> model_name;
        send(ac, model_name);
    }
    
    return 0;
}