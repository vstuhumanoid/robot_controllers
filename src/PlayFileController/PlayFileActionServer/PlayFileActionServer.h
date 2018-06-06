//
// Created by humanoid on 27.04.18.
//

#ifndef ROBOT_CONTROLLERS_PLAYFILEACTIONSERVER_H
#define ROBOT_CONTROLLERS_PLAYFILEACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers/PlayFileAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <robot_msgs/JointsCommand.h>
#include <robot_msgs/JointsMode.h>
#include <robot_msgs/JointsParams.h>
#include <robot_controllers/MotionParams.h>

#include <string>
#include <mutex>

class PlayFileActionServer
{
public:

    PlayFileActionServer(ros::NodeHandle& nh, std::string server_name, int execute_rate);

private:

    bool initializeGoal(robot_controllers::PlayFileGoalConstPtr goal);
    void execute_step();
    void execute_cb(const robot_controllers::PlayFileGoalConstPtr &goal);
    void joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg);
    //void preemptCB();

    sensor_msgs::JointState jointState_;
    robot_msgs::JointsParams jointsParams_;
    robot_msgs::JointsCommand jointsCommand_;
    robot_msgs::JointsMode jointsMode_;

    std::mutex locker_;

    ros::NodeHandle& nh_;

    actionlib::SimpleActionServer<robot_controllers::PlayFileAction> as_;
    robot_controllers::PlayFileResult result_;
    robot_controllers::PlayFileFeedback feedback_;

    actionlib::SimpleActionClient<robot_controllers::TransitionAction> ac_;

    ros::Publisher joints_command_publisher_;
    ros::Publisher joints_mode_publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber params_subscriber_;

    const int execute_rate_;
};


#endif //ROBOT_CONTROLLERS_PLAYFILEACTIONSERVER_H
