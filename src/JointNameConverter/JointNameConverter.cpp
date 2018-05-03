//
// Created by humanoid on 27.04.18.
//

#include "JointNameConverter.h"

using namespace std;

void JointNameConverter::init()
{
    // чтение настроек, будет выполнено один раз при первом вызове
    ros::NodeHandle h;
    map<string, int> map;
    if(!h.getParam("map_number_to_name", map))
        ROS_ERROR("cant get param");

    for(auto it : map)
    {
        numberToName[it.second] = it.first;
        nameToNumber[it.first] = it.second;
    }
}

bool JointNameConverter::getName(int number, std::string &name)
{
    if(numberToName.find(number) == numberToName.end())
        return false;
    name = numberToName[number];
    return true;
}

bool JointNameConverter::getNumber(std::string name, int &number)
{
    if(nameToNumber.find(name) == nameToNumber.end())
        return false;
    number = nameToNumber[name];
    return true;
}

JointNameConverter::JointNameConverter()
{
    init();
}
