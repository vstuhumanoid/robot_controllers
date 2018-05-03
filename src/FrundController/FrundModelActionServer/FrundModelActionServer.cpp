//
// Created by skorikov on 25.04.18.
//

#include "FrundModelActionServer.h"

using namespace robot_controllers;
using namespace std;
using namespace robot_msgs;
using namespace sensor_msgs;
using namespace message_filters;

void wtf(const sensor_msgs::JointStateConstPtr &state,
         const sensor_msgs::ImuConstPtr &imu,
         const robot_msgs::FeetSensorsConstPtr &feet,
         /*robot_msgs::JointsSupplyStateConstPtr &supply*/
        const robot_msgs::JointsSupplyStateConstPtr &fff )
{

}

FrundModelActionServer::FrundModelActionServer(ros::NodeHandle& nh, string server_name) :
    as_(nh, server_name, boost::bind(&FrundModelActionServer::execute_cb, this, _1), false),
    nh_(nh),
    ac_("transition_controller", true)
{
    joints_mode_publisher_ = nh_.advertise<JointsMode>("/joints/set_mode", 100);
    joints_command_publisher_ = nh_.advertise<JointsCommand>("/joints/commands", 1000);
    //state_subscriber_ = nh_.subscribe("/joints/state", 1000, &FrundModelActionServer::joints_state_cb, this);
    params_subscriber_ = nh_.subscribe("/joints/get_params", 100, &FrundModelActionServer::joints_params_cb, this);
    motion_params_subscriber_ = nh_.subscribe("/frund/motion_params", 1000, &FrundModelActionServer::motion_params_cb, this);

    joint_state_sub_.subscribe(nh_, "/joints/state", 1000);
    imu_sub_ .subscribe(nh_, "/sensors/imu", 1000);
    feet_sensors_sub_.subscribe(nh_, "/sensors/feet", 1000);
    joint_supply_state_sub_.subscribe(nh_, "/power/joints_state", 100);

    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), joint_state_sub_, imu_sub_, feet_sensors_sub_, joint_supply_state_sub_);
    sync->registerCallback(&FrundModelActionServer::robot_state_cb, this);
    as_.start();


    //auto wtf = boost::bind(&FrundModelActionServer::robot_state_cb, this, _1, _2, _3, _4);
}

bool FrundModelActionServer::initializeGoal(FrundModelGoalConstPtr goal)
{

}

void FrundModelActionServer::execute_transition()
{
    //TODO: вызов перехода (передавать цель или позиции?)
}

void FrundModelActionServer::execute_cb(const FrundModelGoalConstPtr &goal)
{

}

/*void FrundModelActionServer::preemptCB()
{
    // Либо отправлена новая задача, либо текущая задача отменена
    ROS_WARN_STREAM("preempt_cb");
}*/

void FrundModelActionServer::joints_state_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointState_ = *msg;
}

void FrundModelActionServer::joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointsParams_ = *msg;
}

void FrundModelActionServer::motion_params_cb(const robot_controllers::MotionParamsConstPtr &msg)
{
    MotionParams motionParams = *msg;

    //TODO: Передача ФРУНДУ
}

void FrundModelActionServer::robot_state_cb(const sensor_msgs::JointStateConstPtr &state,
                                            const sensor_msgs::ImuConstPtr &imu,
                                            const robot_msgs::FeetSensorsConstPtr &feet,
                                            const robot_msgs::JointsSupplyStateConstPtr &supply)
{

}