//
// Created by humanoid on 09.04.18.
//

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_controllers/TransitionAction.h>
#include <sensor_msgs/JointState.h>
#include <robot_controller_ros/JointsCommand.h>
#include <robot_controller_ros/JointsMode.h>

#include <string>
#include <mutex>

using namespace std;
using namespace robot_controllers;
using namespace robot_controller_ros;

#define EXECUTE_RATE 200
#define CANCEL_ON_NEW

class TransitionActionServer
{
public:
    TransitionActionServer(ros::NodeHandle& nh, string server_name) :
    as_(nh, server_name, boost::bind(&TransitionActionServer::execute_cb, this, _1), false)
    {
        jointsCommandPublisher = nh.advertise<JointsCommand>("joints/commands", 1000);
        jointsModePublisher = nh.advertise<JointsMode>("joints/set_mode", 100);
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
            //TODO: сделать сопоставление по имени джойнта!!!
            if(state.name[i] != goal->names[i])
            {
                ROS_WARN_STREAM("name from goal doesn't match name in state");
                return false;
            }
            double start_angle = state.position[i];
            double dest_angle = goal->positions[i];

            diff_angle = dest_angle - start_angle;

            max_diff = (abs(diff_angle) > max_diff ) ? abs(diff_angle) : max_diff;

            transition_data[i].name = goal->names[i];
            transition_data[i].start_angle = start_angle;
            transition_data[i].dest_angle = dest_angle;
        }

        time_ms = max_diff * 1000 / speed;

        time_ms = time_ms < 1000 ? 1000 : time_ms;

        for(int i = 0; i < size; i++)
        {
            //TODO: как-то получить sendDelay
            transition_data[i].step = diff_angle/(time_ms / 1000 * EXECUTE_RATE); //TODO: ПИЗДЕЦ
            transition_data[i].current_angle = transition_data[i].start_angle;
            transition_data[i].is_end_angle = false;
        }

        locker.unlock();

        joints_in_transition_count = size;

        return true;
    }

    void execute_transition_step()
    {
        jointsCommand.names.clear();
        jointsCommand.positions.clear();
        jointsCommand.pids.clear();
        jointsMode.names.clear();
        jointsMode.modes.clear();
        TypeJointMode mode;
        mode.mode = TypeJointMode::BREAK;

        //for(auto it = transition_data.begin(); it != transition_data.end(); ++it)
        for(auto transition: transition_data)
        {
            bool is_first = transition.second.dest_angle <= transition.second.current_angle
                           && transition.second.dest_angle >= transition.second.start_angle;
            bool is_second = transition.second.dest_angle >=transition.second.current_angle
                            && transition.second.dest_angle <=transition.second.start_angle;

            if(is_first || is_second)
            {
                jointsCommand.names.push_back(transition.second.name);
                jointsCommand.positions.push_back(transition.second.dest_angle);

                jointsMode.names.push_back(transition.second.name);
                jointsMode.modes.push_back(mode);

                if(!transition.second.is_end_angle)
                {
                    joints_in_transition_count--;
                    transition.second.is_end_angle = true;
                }
            }
            else
            {
                jointsCommand.names.push_back(transition.second.name);
                jointsCommand.positions.push_back(transition.second.current_angle);

                transition.second.current_angle += transition.second.step;
            }
        }

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

        short percentage;
        double counter = 0;

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
            percentage = (short)(counter / time_ms * 100);
            feedback_.percentage = percentage;
            as_.publishFeedback(feedback_);
            rate.sleep();
            counter += 1/EXECUTE_RATE;
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
        state = *msg;
        locker.unlock();
        //ROS_INFO("I heard: [%s]", msg->name.size());
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

    sensor_msgs::JointState state;
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
    int speed = 10;      // скорость перехода
    double time_ms;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "transition_controller");
    ROS_INFO("i am started...");
    ros::NodeHandle nh;
    TransitionActionServer action(nh, ros::this_node::getName());

    ros::Subscriber subscriber = nh.subscribe("joints/state", 1000, &TransitionActionServer::joints_state_cb, &action);

    ros::spin();
    return 0;
}

