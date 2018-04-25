//
// Created by humanoid on 25.04.18.
//

#ifndef ROBOT_CONTROLLERS_TRANSITIONACTIONSERVER_H
#define ROBOT_CONTROLLERS_TRANSITIONACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <robot_msgs/JointsCommand.h>
#include <robot_msgs/JointsMode.h>
#include <robot_msgs/JointsParams.h>

#include <string>
#include <mutex>


class TransitionActionServer
{
public:
    TransitionActionServer(ros::NodeHandle& nh, std::string server_name, int execute_rate, double transition_speed);


private:

    bool initializeGoal(robot_controllers::TransitionGoalConstPtr goal);
    void execute_transition_step();
    void execute_cb(const robot_controllers::TransitionGoalConstPtr &goal);
    //void preemptCB();
    void joints_state_cb(const sensor_msgs::JointStateConstPtr &msg);
    void joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg);

    struct PositionData
    {
        std::string name;
        double current_angle;
        double start_angle;
        double dest_angle;
        double step;
        bool is_end_angle;
    };

    std::map<int, PositionData> transition_data;

    int joints_in_transition_count = 0;
    double transition_time;

    sensor_msgs::JointState jointState_;
    robot_msgs::JointsParams jointsParams_;
    robot_msgs::JointsCommand jointsCommand_;
    robot_msgs::JointsMode jointsMode_;

    std::mutex locker_;

    ros::NodeHandle& nh_;
    actionlib::SimpleActionServer<robot_controllers::TransitionAction> as_;
    robot_controllers::TransitionResult result_;
    robot_controllers::TransitionFeedback feedback_;
    ros::Publisher joints_command_publisher_;
    ros::Publisher joints_mode_publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber params_subscriber_;

    const double transition_speed_;
    const int execute_rate_;
};



#endif //ROBOT_CONTROLLERS_TRANSITIONACTIONSERVER_H
