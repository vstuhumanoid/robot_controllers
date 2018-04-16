#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <mutex>
#include <string>
#include <ros/topic.h>

using namespace std;
using namespace robot_controllers;

sensor_msgs::JointState jointState;

void send(actionlib::SimpleActionClient<TransitionAction>& ac) //, int value, int waiting_time)
{
    TransitionGoal goal;


    /*goal.names.push_back("hip_f_joint_right");
    goal.positions.push_back(1.0);

    goal.names.push_back("hip_f_joint_left");
    goal.positions.push_back(0.0);*/

    int size = jointState.name.size();
    goal.names.resize(size);
    goal.positions.resize(size);
    for(int i = 0; i < size; i++)
    {
        goal.names[i] = jointState.name[i];
        goal.positions[i] = 0;
    }

    //int state = 0;

    while(ros::ok())
    {
        /*if(state == 0)
        {
            goal.positions[0] = 1.0;
            goal.positions[1] = 0.0;
            state = 1;
        }
        else
        {
            goal.positions[0] = 0.0;
            goal.positions[1] = 1.0;
            state = 0;
        }*/

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


}

/*void joints_state_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    jointState = *msg;
    isSomething = true;
}*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transition_client");
    actionlib::SimpleActionClient<TransitionAction> ac("transition_controller", true);

    /*int task_count = std::stoi(argv[2]);
    int wait = std::stoi(argv[3]);*/

    ros::NodeHandle nh;
    jointState  = *ros::topic::waitForMessage<sensor_msgs::JointState>("joints/state");

    ROS_INFO("Waiting for action server to start...");
    ac.waitForServer();
    ROS_INFO("Server started");

    //send(ac, task_count, wait);
    send(ac);

    return 0;
}