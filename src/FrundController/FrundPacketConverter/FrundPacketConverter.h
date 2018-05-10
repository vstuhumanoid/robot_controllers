//
// Created by humanoid on 03.05.18.
//

#ifndef ROBOT_CONTROLLERS_FRUND_PACKET_CONVERTER_H
#define ROBOT_CONTROLLERS_FRUND_PACKET_CONVERTER_H

#define KDRIVE 21
#define SIZEDRIVE 72 // 9*8

#include <string>
#include <tuple>
#include "robot_msgs/JointsCommand.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include "robot_msgs/FeetSensors.h"
#include "robot_msgs/JointsSupplyState.h"
#include "robot_msgs/SourcesSupplyState.h"

#include "JointNameConverter.h"

class FrundPacketConverter
{
private:
    JointNameConverter jointConverter;
public:
    robot_msgs::JointsCommand getMessage(char *array);
    void getArray(sensor_msgs::JointState joints, sensor_msgs::Imu imu, robot_msgs::FeetSensors feet,
                  robot_msgs::JointsSupplyState jointsSupply, char *array);
};


#endif //ROBOT_CONTROLLERS_FRUND_PACKET_CONVERTER_H
