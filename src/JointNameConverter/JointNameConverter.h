//
// Created by humanoid on 27.04.18.
//

#ifndef ROBOT_CONTROLLERS_JOINTNAMECONVERTER_H
#define ROBOT_CONTROLLERS_JOINTNAMECONVERTER_H

#include <ros/ros.h>

#include <string>
#include <map>

class JointNameConverter
{
private:
    std::map<int, std::string> numberToName;
    std::map<std::string, int> nameToNumber;
public:
    void init();
    bool getName(int number, std::string & name);
    bool getNumber(std::string name, int & number);
    JointNameConverter();
};


#endif //ROBOT_CONTROLLERS_JOINTNAMECONVERTER_H
