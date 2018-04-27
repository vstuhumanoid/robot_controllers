//
// Created by humanoid on 27.04.18.
//

#include "JointNameConverter.h"

using namespace std;

void JointNameConverter::init()
{
    // чтение настроек, будет выполнено один раз при первом вызове
    ros::NodeHandle h;
    map<string, int> map_, map2;
    map_["a"] = 1;
    map_["b"] = 2;
    h.setParam("map_a_to_b", map_);
    h.getParam("map_a_to_b", map2);

    string mappppp;
    if(!h.getParam("map_number_to_name", map_))
        ROS_ERROR("cant get param");

    for(auto it : map_)
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
