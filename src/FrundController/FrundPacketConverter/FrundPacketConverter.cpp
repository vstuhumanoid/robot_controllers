//
// Created by humanoid on 03.05.18.
//

#include "FrundPacketConverter.h"
using namespace robot_msgs;
using namespace sensor_msgs;
using namespace std;

robot_msgs::JointsCommand FrundPacketConverter::getMessage(char *array)
{
    JointsCommand joints;
    joints.names.resize(KDRIVE);
    joints.positions.resize(KDRIVE);
    joints.pids.resize(KDRIVE);

    for(int i = 0; i < KDRIVE * SIZEDRIVE; i += SIZEDRIVE)
    {
        string name;
        double number;
        memcpy(&number, array + i, sizeof(double));
        if(jointConverter.getName((int)number, name))
        {
            joints.names[i / SIZEDRIVE] = name;

            memcpy(&joints.positions[i / SIZEDRIVE], array + i + 16, sizeof(double));
            memcpy(&joints.pids[i / SIZEDRIVE].p, array + i + 24, sizeof(double));
            memcpy(&joints.pids[i / SIZEDRIVE].i, array + i + 32, sizeof(double));
            memcpy(&joints.pids[i / SIZEDRIVE].d, array + i + 40, sizeof(double));
        }
    }

    return joints;
}

void FrundPacketConverter::getArray(sensor_msgs::JointState joints, sensor_msgs::Imu imu, robot_msgs::FeetSensors feet,
                                    robot_msgs::JointsSupplyState jointsSupply, char *array)
{
    int sizeOfData = joints.name.size() + 6 + 8;
    std::map<int, std::tuple<double, double>> jointsMap;

    double data[sizeOfData];

    for(int i = 0; i < joints.position.size(); i++)
    {
        int number;
        if(jointConverter.getNumber(joints.name[i], number))
            jointsMap[number] = std::tuple<double, double>(joints.position[i], jointsSupply.states[i].Current);
    }

    int i = 0;
    for (auto joint : jointsMap)
        data[i++] = std::get<0>(joint.second);

    data[i++] = imu.linear_acceleration.x;
    data[i++] = imu.linear_acceleration.y;
    data[i++] = imu.linear_acceleration.z;
    data[i++] = imu.orientation.x;
    data[i++] = imu.orientation.y;
    data[i++] = imu.orientation.z;

    data[i++] = feet.left.uch0;
    data[i++] = feet.left.uch1;
    data[i++] = feet.left.uch2;
    data[i++] = feet.left.uch3;
    data[i++] = feet.right.uch0;
    data[i++] = feet.right.uch1;
    data[i++] = feet.right.uch2;
    data[i++] = feet.right.uch3;

    double driverPower;
    for(auto supply : jointsSupply.states)
        driverPower += supply.Current * supply.Voltage;
    data[i++] = driverPower;

    for (auto joint : jointsMap)
        data[i++] = std::get<1>(joint.second);

    for(int j = 0; j < sizeOfData; j++)
        memcpy(array+(j * sizeof(double)), &data[j], sizeof(double));
}
