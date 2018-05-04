//
// Created by skorikov on 25.04.18.
//

#include "FrundModelActionServer.h"

using namespace robot_controllers;
using namespace std;
using namespace robot_msgs;
using namespace sensor_msgs;
using namespace message_filters;

FrundModelActionServer::FrundModelActionServer(ros::NodeHandle& nh, string server_name, int execute_rate) :
    as_(nh, server_name, boost::bind(&FrundModelActionServer::execute_cb, this, _1), false),
    nh_(nh),
    transition_ac_("transition_controller", true),
    execute_rate_(execute_rate)
{
    int frund_port, frund_runner_port;
    std::string frund_runner_address;
    if(!nh_.getParam("frund_port", frund_port)
       || !nh_.getParam("frund_runner_address", frund_runner_address)
        || !nh_.getParam("frund_runner_port", frund_runner_port))
    {
        ROS_ERROR("Not enough params!");
        exit(1);
    }

    if(!frundGateway_.init((uint16_t)frund_port, (uint16_t)frund_runner_port, frund_runner_address))
    {
        ROS_ERROR("Connection init error!");
        exit(1);
    }

    joints_mode_publisher_ = nh_.advertise<JointsMode>("/joints/set_mode", 100);
    joints_command_publisher_ = nh_.advertise<JointsCommand>("/joints/commands", 1000);

    params_subscriber_ = nh_.subscribe("/joints/get_params", 100, &FrundModelActionServer::joints_params_cb, this);
    motion_params_subscriber_ = nh_.subscribe("/frund/motion_params", 1000, &FrundModelActionServer::motion_params_cb, this);

    joint_state_sub_.subscribe(nh_, "/joints/state", 1000);
    imu_sub_ .subscribe(nh_, "/sensors/imu", 1000);
    feet_sensors_sub_.subscribe(nh_, "/sensors/feet", 1000);
    joint_supply_state_sub_.subscribe(nh_, "/power/joints_state", 100);

    sync_ = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), joint_state_sub_, imu_sub_, feet_sensors_sub_, joint_supply_state_sub_);
    sync_->registerCallback(&FrundModelActionServer::robot_state_cb, this);
    as_.start();
}

bool FrundModelActionServer::initializeGoal(FrundModelGoalConstPtr goal)
{
    if(!frundGateway_.RunModel(goal->model))
        return false;
    current_model_ = goal->model;
    return true;
}

void FrundModelActionServer::execute_cb(const FrundModelGoalConstPtr &goal)
{
    ros::Rate rate(execute_rate_);
    ROS_INFO_STREAM("New goal received");

    if(!initializeGoal(goal))
    {
        as_.setAborted(result_, "Invalid goal");
        ROS_WARN("Invalid goal. Ignoring...");
        return;
    }


    int state = 1;
    char packet[PACKET_SIZE];
    JointsCommand command;
    TypeJointMode mode;
    mode.mode = TypeJointMode::TRACE;

    while(ros::ok())
    {
        // Проверяем новую задачу или отмену
        if(as_.isPreemptRequested() && as_.isNewGoalAvailable())
        {
            // Текущая задача была "вытеснена" (preempted) и доступна новая - значит поставлена новая задача
            // TODO: Начало следующей задачи
            // Текущая задача будет преврана и начата новая
            ROS_INFO("Goal preempted");
            as_.setPreempted(result_, "New task received");
            return;
        }
        else if(as_.isPreemptRequested())
        {
            // Текущая задача была "вытеснена" (preempted) и нет новой - значит текущая задача просто прервана
            // Прерываем текущую задачу
            ROS_WARN("Current goal cancelled");
            as_.setAborted(result_, "Cancelled");
            stop_work("Current goal cancelled");
            return;
        }

        if(state == 1)      // подготовка к переходу
        {
            if(frundGateway_.ReceivePacket(packet, PACKET_SIZE))
            {
                JointsCommand command = frundPacketConverter_.getMessage(packet);
                TransitionGoal transition_goal;
                transition_goal.names = command.names;
                transition_goal.positions = command.positions;
                transition_ac_.sendGoal(transition_goal);
                state = 2;
            }
        }

        // переход в позицию (отслеживать кансел)
        if(state == 2)
        {
            bool is_ok = transition_ac_.waitForResult(ros::Duration(1));
            if(is_ok)
            {
                auto transition_state = transition_ac_.getState();
                if(transition_state.state_ == transition_state.ABORTED)
                {
                    // переход отменен
                    as_.setAborted(result_, "Transition cancelled");
                    stop_work("Transition cancelled");
                    return;
                }
                else if(transition_state.state_ == transition_state.SUCCEEDED)
                {
                    ROS_INFO("Transition to init position is completed");
                    state = 3;
                }
            }
        }

        if(state == 3)
        {
            jointsMode_.names = command.names;
            jointsMode_.modes.resize(jointsMode_.names.size());
            for(int i = 0; i < jointsMode_.modes.size(); i++)
                jointsMode_.modes[i] = mode;
            joints_mode_publisher_.publish(jointsMode_);
            state = 4;
        }

        // основной цикл управления (отслеживать кансел)
        if(state == 4)
        {
            locker_.lock();
            frundPacketConverter_.getArray(jointState_, imu_, feetSensors_, jointsSupplyState_, packet);
            locker_.unlock();
            frundGateway_.SendPacket(packet, PACKET_SIZE);
            state = 5;
        }

        if(state == 5)
        {
            if(frundGateway_.ReceivePacket(packet, PACKET_SIZE))
            {
                command = frundPacketConverter_.getMessage(packet);
                joints_command_publisher_.publish(command);
                state = 4;
            }
        }

        rate.sleep();
    }

    if(!ros::ok())
    {
        // Если нода завершилась, то прерываем текущую задачу
        as_.setAborted(result_, "Server node is stopped");
        stop_work("Server node is stopped");
        return;
    }
}

void FrundModelActionServer::stop_work(string message)
{
    ROS_INFO_STREAM("Stopping...    " + message);
    frundGateway_.StopModel();

    TypeJointMode mode;
    mode.mode = TypeJointMode::TRACE;
    jointsMode_.names = jointState_.name;
    jointsMode_.modes.resize(jointState_.name.size());
    for(int i = 0; i < jointsMode_.modes.size(); i++)
        jointsMode_.modes[i] = mode;
    joints_mode_publisher_.publish(jointsMode_);
}

void FrundModelActionServer::joints_params_cb(const robot_msgs::JointsParamsConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointsParams_ = *msg;
}

void FrundModelActionServer::motion_params_cb(const robot_controllers::MotionParamsConstPtr &msg)
{
    MotionParams motionParams = *msg;
    frundGateway_.SendParams(current_model_, motionParamsToString(motionParams));
}

string FrundModelActionServer::motionParamsToString(MotionParams params)
{
    //TODO: преобразовать параметры в строку
    string params_string;
    params_string += "1 " + std::to_string(params.velocity_x) + " " + std::to_string(params.velocity_y) + " 1";
    params_string += "\r\n" + std::to_string(params.step_height);
    params_string += "\r\n" + std::to_string(params.step_length);
    params_string += "\r\n" + std::to_string(params.com_height);
    params_string += "\r\n" + std::to_string(params.angular_velocity);
    return params_string;
}

void FrundModelActionServer::robot_state_cb(const sensor_msgs::JointStateConstPtr &state,
                                            const sensor_msgs::ImuConstPtr &imu,
                                            const robot_msgs::FeetSensorsConstPtr &feet,
                                            const robot_msgs::JointsSupplyStateConstPtr &supply)
{
    lock_guard<mutex> lock(locker_);
    jointState_ = *state;
    imu_ = *imu;
    feetSensors_ = *feet;
    jointsSupplyState_ = *supply;
}