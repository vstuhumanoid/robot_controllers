//
// Created by humanoid on 25.04.18.
//

#include "TransitionActionServer.h"

using namespace std;
using namespace robot_controllers;
using namespace robot_msgs;

TransitionActionServer::TransitionActionServer(ros::NodeHandle& nh, string server_name, int execute_rate, double transition_speed) :
    as_(nh, server_name, boost::bind(&TransitionActionServer::execute_cb, this, _1), false),
    execute_rate_(execute_rate),
    transition_speed_(transition_speed),
    nh_(nh)
{
    joints_mode_publisher_ = nh_.advertise<JointsMode>("/joints/set_mode", 100);
    joints_command_publisher_ = nh_.advertise<JointsCommand>("/joints/commands", 1000);
    state_subscriber_ = nh_.subscribe("/joints/state", 1000, &TransitionActionServer::joints_state_cb, this);
    params_subscriber_ = nh_.subscribe("/joints/get_params", 100, &TransitionActionServer::joints_params_cb, this);

    //as_.registerPreemptCallback(boost::bind(&TransitionActionServer::preemptCB, this));
    as_.start();
}

bool TransitionActionServer::initializeGoal(TransitionGoalConstPtr goal)
{
    double max_diff = 0;
    int size;

    if((size = goal->names.size()) != goal->positions.size())
    {
        ROS_WARN_STREAM("names count != positions count");
        return false;
    }

    locker_.lock();

    double diff_angle;

    for(int i = 0; i < size; i++)
    {
        double start_angle;

        bool isExist = false;
        bool isEnabled = true;
        for(int  j = 0; j < jointState_.name.size(); j++)
        {
            if(jointState_.name[j] == goal->names[i])
            {
                start_angle = jointState_.position[j];
                isExist = true;
            }

            if(jointsParams_.names[j] == goal->names[i])
                isEnabled = jointsParams_.enabled[j];
        }

        if(!isExist)
        {
            ROS_WARN_STREAM("name from goal doesn't match name in state");
            return false;
        }

        //double start_angle = state.position[i];
        double dest_angle = goal->positions[i];

        diff_angle = dest_angle - start_angle;

        if(isEnabled)
            max_diff = (abs(diff_angle) > max_diff ) ? abs(diff_angle) : max_diff;

        transition_data[i].name = goal->names[i];
        transition_data[i].start_angle = start_angle;
        transition_data[i].dest_angle = dest_angle;

        //ROS_INFO_STREAM("diff angle for " << transition_data[i].name << " : " << diff_angle);
    }

    transition_time = max_diff / transition_speed_;
    transition_time = transition_time < 5 ? 5 : transition_time;

    jointsMode_.names.resize(size);
    jointsMode_.modes.resize(size);

    jointsCommand_.names.resize(size);
    jointsCommand_.positions.resize(size);

    //ROS_INFO_STREAM("transition time: " << transition_time << "sec");

    TypeJointMode mode;
    mode.mode = TypeJointMode::TRACE;

    for(int i = 0; i < size; i++)
    {
        diff_angle = transition_data[i].dest_angle - transition_data[i].start_angle;
        transition_data[i].step = diff_angle / (transition_time * execute_rate_);
        transition_data[i].current_angle = transition_data[i].start_angle;
        transition_data[i].is_end_angle = false;
        jointsMode_.names[i] = transition_data[i].name;
        jointsMode_.modes[i] = mode;

        //ROS_INFO_STREAM("transition step for " << transition_data[i].name << " : " << transition_data[i].step << "rad");
    }

    locker_.unlock();
    joints_mode_publisher_.publish(jointsMode_);
    joints_in_transition_count = size;

    return true;
}

void TransitionActionServer::execute_transition_step()
{
    TypeJointMode mode;
    mode.mode = TypeJointMode::BREAK;

    int i = 0;

    for(auto& transition: transition_data)
    {
        bool is_first = transition.second.dest_angle <= transition.second.current_angle
                        && transition.second.dest_angle >= transition.second.start_angle;
        bool is_second = transition.second.dest_angle >= transition.second.current_angle
                         && transition.second.dest_angle <= transition.second.start_angle;

        if(is_first || is_second)
        {
            jointsCommand_.names[i] = transition.second.name;
            jointsCommand_.positions[i] = transition.second.dest_angle;

            jointsMode_.names[i] = transition.second.name;
            jointsMode_.modes[i] = mode;

            if(!transition.second.is_end_angle)
            {
                joints_in_transition_count--;
                transition.second.is_end_angle = true;
            }
        }
        else
        {
            jointsCommand_.names[i] = transition.second.name;
            jointsCommand_.positions[i] = transition.second.current_angle;
            transition.second.current_angle += transition.second.step;
            //ROS_INFO_STREAM("Position for " << transition.second.name << ": " << transition.second.current_angle);
        }

        i++;
    }

    jointsCommand_.header.stamp = ros::Time::now();
    joints_command_publisher_.publish(jointsCommand_);
    joints_mode_publisher_.publish(jointsMode_);    //что если пустое?
}

void TransitionActionServer::execute_cb(const TransitionGoalConstPtr &goal)
{
    ros::Rate rate(execute_rate_);
    ROS_INFO_STREAM("New goal received");

    // Инициализируем задачу
    if(!initializeGoal(goal))
    {
        as_.setAborted(result_, "Invalid goal");
        ROS_WARN("Invalid goal. Ignoring...");
        return;
    }

    float percentage;
    float counter = 0;

    // Выполняем задачу
    while(joints_in_transition_count != 0)
    {
        if(!ros::ok())
        {
            // Если нода завершилась, то прерываем текущую задачу
            //TODO: перевести моторы в стоп?
            as_.setAborted(result_, "Server node is stopped");
            ROS_INFO("Stopping...");
            return;
        }

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
            ROS_WARN("Current goal canceled");
            as_.setAborted(result_, "Canceled");
            return;
        }

        // Выполняем задачу
        execute_transition_step();
        percentage = counter / transition_time * 100;
        feedback_.percentage = percentage;
        as_.publishFeedback(feedback_);
        rate.sleep();
        counter += 1 / execute_rate_;
    }

    // Готово
    //TODO: перевести моторы в стоп?
    as_.setSucceeded(result_, "Transition completed");
    ROS_INFO_STREAM("Transition completed");
}

/*void TransitionActionServer::preemptCB()
{
    // Либо отправлена новая задача, либо текущая задача отменена
    ROS_WARN_STREAM("preempt_cb");
}*/

void TransitionActionServer::joints_state_cb(const sensor_msgs::JointStateConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointState_ = *msg;
}

void TransitionActionServer::joints_params_cb(const JointsParamsConstPtr &msg)
{
    lock_guard<mutex> lock(locker_);
    jointsParams_ = *msg;
}