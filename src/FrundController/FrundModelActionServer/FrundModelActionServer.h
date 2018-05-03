//
// Created by skorikov on 25.04.18.
//

#ifndef ROBOT_CONTROLLERS_FRUNDMODELACTIONSERVER_H
#define ROBOT_CONTROLLERS_FRUNDMODELACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers/FrundModelAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_controllers/TransitionAction.h>

#include <sensor_msgs/JointState.h>
#include <robot_msgs/JointsCommand.h>
#include <robot_msgs/JointsMode.h>
#include <robot_msgs/JointsParams.h>
#include <sensor_msgs/Imu.h>
#include <robot_msgs/FeetSensors.h>
#include <robot_msgs/JointsSupplyState.h>
#include <robot_controllers/MotionParams.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <mutex>

#include "FrundPacketConverter/FrundPacketConverter.h"

#include <std_msgs/Int32.h>

//#define  WTF sensor_msgs::JointState
//#define WTF_PTR sensor_msgs::JointStateConstPtr

//#define WTF robot_msgs::JointsSupplyState
//#define WTF_PTR robot_msgs::JointsSupplyStateConstPtr

class FrundModelActionServer
{
public:

    FrundModelActionServer(ros::NodeHandle& nh, std::string server_name);

private:

    bool initializeGoal(robot_controllers::FrundModelGoalConstPtr goal);
    void execute_transition();
    void execute_cb(const robot_controllers::FrundModelGoalConstPtr &goal);
    //void preemptCB();
    void joints_state_cb(const sensor_msgs::JointStateConstPtr &msg);
    void joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg);
    void motion_params_cb(const robot_controllers::MotionParamsConstPtr &msg);

    void robot_state_cb(const sensor_msgs::JointStateConstPtr &state, const sensor_msgs::ImuConstPtr &imu,
                        const robot_msgs::FeetSensorsConstPtr &feet, const robot_msgs::JointsSupplyStateConstPtr &supply);


    sensor_msgs::JointState jointState_;
    robot_msgs::JointsParams jointsParams_;
    robot_msgs::JointsCommand jointsCommand_;
    robot_msgs::JointsMode jointsMode_;
    robot_msgs::JointsSupplyState jointsSupplyState_;
    sensor_msgs::Imu imu_;
    robot_msgs::FeetSensors feetSensors_;

    std::mutex locker_;

    ros::NodeHandle& nh_;

    actionlib::SimpleActionServer<robot_controllers::FrundModelAction> as_;
    robot_controllers::FrundModelResult result_;
    robot_controllers::FrundModelFeedback feedback_;

    actionlib::SimpleActionClient<robot_controllers::TransitionAction> ac_;

    ros::Publisher joints_command_publisher_;
    ros::Publisher joints_mode_publisher_;
    //ros::Subscriber state_subscriber_;
    ros::Subscriber params_subscriber_;
    ros::Subscriber motion_params_subscriber_;

    message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    message_filters::Subscriber<robot_msgs::FeetSensors> feet_sensors_sub_;
    message_filters::Subscriber<robot_msgs::JointsSupplyState> joint_supply_state_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, sensor_msgs::Imu, robot_msgs::FeetSensors, robot_msgs::JointsSupplyState> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;
};


#endif //ROBOT_CONTROLLERS_FRUNDMODELACTIONSERVER_H
