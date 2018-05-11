#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <string>
#include <ros/topic.h>
#include <iostream>

using namespace std;
using namespace robot_controllers;

sensor_msgs::JointState jointState;

void send(actionlib::SimpleActionClient<TransitionAction>& ac, float goal_angle)
{
    TransitionGoal goal;

    int size = jointState.name.size();
    goal.names.resize(size);
    goal.positions.resize(size);
    for(int i = 0; i < size; i++)
    {
        goal.names[i] = jointState.name[i];
        goal.positions[i] = goal_angle;
    }

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

    ros::init(argc, argv, argv[1]);                 //"transition_client");
    actionlib::SimpleActionClient<TransitionAction> ac("transition_controller", true);

    /*int task_count = std::stoi(argv[2]);
    int wait = std::stoi(argv[3]);*/

    ros::NodeHandle nh;
    jointState  = *ros::topic::waitForMessage<sensor_msgs::JointState>("joints/state");

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Server started");

    float goal_angle = 0;
    while(ros::ok())
    {
        string goal;
        //send(ac, task_count, wait);
        cout << "Enter goal: ";
        cin >> goal;
        try
        {
            goal_angle = stof(goal, nullptr);
            send(ac, goal_angle);
        }
        catch (exception e)
        {
            cout << "Invalid goal - " << e.what() << endl;
        }
    }

    return 0;
}