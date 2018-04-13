#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>

using namespace std;
using namespace robot_controllers;

void send(actionlib::SimpleActionClient<TransitionAction>& ac) //, int value, int waiting_time)
{
    TransitionGoal goal;
    goal.names.push_back("1");
    goal.positions.push_back(30.0);

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
    ros::init(argc, argv, "transition_client");
    actionlib::SimpleActionClient<TransitionAction> ac("transition_controller", true);

    /*int task_count = std::stoi(argv[2]);
    int wait = std::stoi(argv[3]);*/

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Server started");

    //send(ac, task_count, wait);
    send(ac);

    return 0;
}