//
// Created by humanoid on 09.04.18.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <robot_controller_ros/JointsCommand.h>
#include <robot_controller_ros/JointsMode.h>
#include <robot_controller_ros/JointsParams.h>

#include <string>
#include <mutex>

using namespace std;
using namespace robot_controllers;
using namespace robot_controller_ros;

#define EXECUTE_RATE 100  //100

class TransitionActionServer
{
public:
    TransitionActionServer(ros::NodeHandle& nh, string server_name) :
    as_(nh, server_name, boost::bind(&TransitionActionServer::execute_cb, this, _1), false)
    {
        jointsModePublisher = nh.advertise<JointsMode>("joints/set_mode", 100);
        jointsCommandPublisher = nh.advertise<JointsCommand>("joints/commands", 1000);

        as_.registerPreemptCallback(boost::bind(&TransitionActionServer::preemptCB, this));
        as_.start();
    }

    bool initializeGoal(TransitionGoalConstPtr goal)
    {
        double max_diff = 0;
        int size;

        if((size = goal->names.size()) != goal->positions.size())
        {
            ROS_WARN_STREAM("names count != positions count");
            return false;
        }

        locker.lock();

        double diff_angle;

        for(int i = 0; i < size; i++)
        {
            double start_angle;

            bool isExist = false;
            bool isEnabled = true;
            for(int  j = 0; j < jointState.name.size(); j++)
            {
                if(jointState.name[j] == goal->names[i])
                {
                    start_angle = jointState.position[j];
                    isExist = true;
                }

                //if(jointsParams.names[j] == goal->names[i])
                //    isEnabled = jointsParams.enabled[j];
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

        transition_time = max_diff / transition_speed;
        transition_time = transition_time < 5 ? 5 : transition_time;

        jointsMode.names.resize(size);
        jointsMode.modes.resize(size);

        jointsCommand.names.resize(size);
        jointsCommand.positions.resize(size);

        //ROS_INFO_STREAM("transition time: " << transition_time << "sec");

        TypeJointMode mode;
        mode.mode = TypeJointMode::TRACE;

        for(int i = 0; i < size; i++)
        {
            diff_angle = transition_data[i].dest_angle - transition_data[i].start_angle;
            transition_data[i].step = diff_angle / (transition_time * EXECUTE_RATE);
            transition_data[i].current_angle = transition_data[i].start_angle;
            transition_data[i].is_end_angle = false;
            jointsMode.names[i] = transition_data[i].name;
            jointsMode.modes[i] = mode;

            //ROS_INFO_STREAM("transition step for " << transition_data[i].name << " : " << transition_data[i].step << "rad");
        }

        locker.unlock();

        jointsModePublisher.publish(jointsMode);

        joints_in_transition_count = size;

        return true;
    }

    void execute_transition_step()
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
                jointsCommand.names[i] = transition.second.name;
                jointsCommand.positions[i] = transition.second.dest_angle;

                jointsMode.names[i] = transition.second.name;
                jointsMode.modes[i] = mode;

                if(!transition.second.is_end_angle)
                {
                    joints_in_transition_count--;
                    transition.second.is_end_angle = true;
                }
            }
            else
            {
                jointsCommand.names[i] = transition.second.name;
                jointsCommand.positions[i] = transition.second.current_angle;
                transition.second.current_angle += transition.second.step;
                //ROS_INFO_STREAM("Position for " << transition.second.name << ": " << transition.second.current_angle);
            }

            i++;
        }

        jointsCommand.header.stamp = ros::Time::now();
        jointsCommandPublisher.publish(jointsCommand);
        jointsModePublisher.publish(jointsMode);    //что если пустое?
    }

    void execute_cb(const TransitionGoalConstPtr &goal)
    {
        ros::Rate rate(EXECUTE_RATE);
        ROS_INFO_STREAM("execute_cb: New goal ");

        // Инициализируем задачу
        if(!initializeGoal(goal))
        {
            as_.setAborted(result_, "Invalid goal");
            ROS_INFO("Invalid goal. Ignoring...");
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
                ROS_WARN("New goal available, but current is not finished. Cancel old, start new");
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
            counter += 1 / EXECUTE_RATE;
        }

        // Готово
        //TODO: перевести моторы в стоп?
        as_.setSucceeded(result_, "Transition completed");
        ROS_INFO_STREAM("Transition completed");
    }

    void preemptCB()
    {
        // Либо отправлена новая задача, либо текущая задача отменена
        ROS_WARN_STREAM("preempt_cb");
    }

    void joints_state_cb(const sensor_msgs::JointStateConstPtr &msg)
    {
        locker.lock();
        jointState = *msg;
        locker.unlock();
    }

    void joints_params_cb(const JointsParamsConstPtr &msg)
    {
        locker.lock();
        jointsParams = *msg;
        locker.unlock();
    }

    void setSpeed(double speed)
    {
        transition_speed = speed;
    }

protected:

    struct PositionData
    {
        string name;
        double current_angle;
        double start_angle;
        double dest_angle;
        double step;
        bool is_end_angle;
    };

    std::map<int, PositionData> transition_data;

    int joints_in_transition_count = 0;

    sensor_msgs::JointState jointState;
    JointsParams jointsParams;
    mutex locker;
    JointsCommand jointsCommand;
    JointsMode jointsMode;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot_controllers::TransitionAction> as_;
    TransitionResult result_;
    TransitionFeedback feedback_;
    ros::Publisher jointsCommandPublisher;
    ros::Publisher jointsModePublisher;

    //TODO: задавать через ros_param
    double transition_speed = 0.2;      // скорость перехода
    double transition_time;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "transition_controller");
    ROS_INFO("i am started...");
    ros::NodeHandle nh;

    TransitionActionServer action(nh, ros::this_node::getName());

    double speed;
    if(nh.getParam("speed", speed))
        action.setSpeed(speed);

    ros::Subscriber stateSubscriber = nh.subscribe("joints/state", 1000, &TransitionActionServer::joints_state_cb, &action);
    ros::Subscriber paramsSubscriber = nh.subscribe("joints/get_params", 100, &TransitionActionServer::joints_params_cb, &action);

    ros::spin();
    return 0;
}

